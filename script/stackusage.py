#! /usr/bin/env python3

"""stackusage.py: A CLI script to perform static stack usage analysis
and export callgraphs

"""

import argparse

import pydot

# This is the indentation delmiter that indicates the total stack
# depth for a given line in a cflow output graph
DELIMITER = '    '


class FunctionCall:
    """A container class which represents a function call, its children,
    and the amount of stack space allocated when the function is invoked

    """

    def __init__(self, name, local_stack):
        self._children = {}
        self.name = name
        self._local_stack = local_stack

        self.node = pydot.Node(self.name)

    def add_child(self, call):
        """ Associates a child function with this (parent) function call

        :param call: FunctionCall - The child call to add
        :returns: None

        """
        if call not in self._children.values():
            self._children[call.name] = call

    @property
    def max_call_graph(self):
        """Returns a string containing a directed graph in DOT format which
        can be compiled by graphviz

        :rtype: str

        """
        graph = pydot.Dot(graph_type='digraph')

        for node in set(self.gv_nodes):
            graph.add_node(node)

        # Perform an edge reduce to clean up the graph. In order to do
        # this effectively, we have to actually build a graph, then
        # iterate over the set of it's nodes to create a duplicate
        # (this is a limitation of pydot as it needs to know the graph
        # directionality to check edge equality)
        tmp_graph = pydot.Dot(graph_type='digraph')
        for edge in self.gv_edges:
            tmp_graph.add_edge(edge)

        for edge in set(tmp_graph.get_edge_list()):
            graph.add_edge(edge)

        return graph

    @property
    def gv_nodes(self):
        """Returns a list of all graph nodes below this function call level

        :returns: A list of pydot nodes
        :rtype: list[pydot.Node]

        """
        nodes = []
        nodes.append(self.node)
        for _, child in self._children.items():
            nodes.extend(child.gv_nodes)

        return nodes

    @property
    def gv_edges(self):
        """Returns a list of all graph edges below this function call level

        :returns: A list of pydot edges
        :rtype: list[pydot.Edge]

        """
        edges = []
        for _, child in self._children.items():
            edges.append(pydot.Edge(self.node, child.node))
            edges.extend(child.gv_edges)
        return edges

    @property
    def max_call_str(self):
        """ Returns a string graphic representing the total stack usage

        :rtype: str

        """
        result = ''
        total = 0
        for dep, (func, stk) in enumerate(self.max_call_stack):
            total += stk
            result += '{:3} |{}>{}[{}]\n'.format(total, dep*'-', func, stk)

        return result

    @property
    def max_call_stack(self):
        """ Recursively calculates the total maximum stack depth

        :returns: a list which details the call stack with the largest
        allocation
        :rtype: list

        """

        if not self._children:
            return [(self.name, self._local_stack)]
        else:
            return ([(self.name, self._local_stack)] +
                    max(self._children.values()).max_call_stack)

    def __str__(self):
        return self.name

    @property
    def max_stack_depth(self):
        """ Recursively calculates the total maximum stack depth of a function

        :returns:
        :rtype:

        """

        if not self._children:
            return self._local_stack
        else:
            return max([x.max_stack_depth for x
                        in self._children.values()]) + self._local_stack

    def __lt__(self, other):
        return self.max_stack_depth < other.max_stack_depth


def parse_stack_data(data):
    """Parses a combined gcc -fstack-usage report

    :param data: str - gcc -fstack-usage report
    :returns: a dictionary in the form {fn_name: frame_stack_depth}
    :rtype: dict
    """

    fn_dict = {}

    for line in data.splitlines():
        # TODO: Make use of the memory class (dynamic/etc) to
        # determine accuracy percentage
        func, amt, _ = line.split()

        *_, funcname = func.split(':')

        fn_dict[funcname] = FunctionCall(funcname, int(amt))

    return fn_dict


def proces_flow_data(fn_dict, flowdata):
    """Integrates gcc function stack usage information with a cflow function call graph

    :param fn_dict: dict - function dictionary produced by `parse_stack_data`
    :param flowdata: str - cflow output file data
    :returns: A FunctionCall object for the program entrypoint (usually main())
    :rtype: FunctionCall

    """
    _stack = []

    # Parse cflow file
    for line in flowdata.splitlines():
        # Local function depth
        l_dep = 0

        # Figure out how deep in the call stack we are by splitting on
        # the delimiter and counting the number of empty strings at
        # the front of the resulting list
        ldata = line.split(DELIMITER)
        while not ldata[l_dep]:
            l_dep += 1

        # re-split remaining data on whitespace
        fn_info = ' '.join(ldata[l_dep:]).split()

        # Pluck out the function name
        fn_name = fn_info[0].strip("()")

        # If we already know about this function, use the copy we
        # already have, otherwise, create a function instance with a
        # zero depth stack.
        # NOTE: We assume zero depth as we don't have enough info to
        # determine actual usage (could be an asm or stdlib function)
        if fn_name in fn_dict:
            l_fn = fn_dict[fn_name]
        else:
            l_fn = FunctionCall(fn_name, 0)
            fn_dict[fn_name] = l_fn

        # If the call stack is empty, make the initial entry,
        # otherwise, track where we should be in the stack via l_dep
        # and ensure that we maintain proper function lineage
        if not _stack:
            _stack.append(l_fn)
        elif l_dep == len(_stack):
            _stack[-1].add_child(l_fn)
            _stack.append(l_fn)
        else:
            while l_dep < len(_stack):
                _stack.pop()

            if l_fn in _stack:
                # TODO: add recursion support and handle in max depth
                # calculators
                print("Recursion detected in fn: {}!".format(l_fn.name))
                continue

            _stack[-1].add_child(l_fn)
            _stack.append(l_fn)

    return _stack[0]


def stackusage(args):
    """Entry point for the application logic

    :param args: dict - argparse arguments
    :returns: None

    """

    with open(args.stack_file) as fil:
        stackdata = fil.read()

    with open(args.cflow_file) as fil:
        flowdata = fil.read()

    fn_dict = parse_stack_data(stackdata)
    pgm_entry = proces_flow_data(fn_dict, flowdata)

    if args.analyze:
        print(pgm_entry.max_call_str)
        print(
            "Estimated maximum stack depth: {} bytes"
            .format(pgm_entry.max_stack_depth)
        )

    if args.callgraph:
        call_graph = pgm_entry.max_call_graph
        call_graph.write_pdf(args.callgraph)


def main():
    """Entry point for command line parsing logic

    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stack', '-s',
                        action='store', dest='stack_file',
                        required=True,
                        help='Aggregated stack usage file')

    parser.add_argument('--flow', '-f',
                        action='store', dest='cflow_file',
                        required=True,
                        help='cflow output file')

    parser.add_argument('--analyze', '-a',
                        action='store_true', dest='analyze',
                        help='Perform a worst-case stack analysis')

    parser.add_argument('--callgraph', '-c',
                        action='store', dest='callgraph',
                        help='Generate a graphviz callgraph in PDF format')

    args = parser.parse_args()

    stackusage(args)

if __name__ == '__main__':
    main()
