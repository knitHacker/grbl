#include "io.h"

// dummy register variables
volatile io_sim_t io={{0}};


static port_monitor_fp io_portfuncs[SIM_PORT_COUNT] = {0}; 

void io_default_port_access(uint8_t port) { /*nop*/; } 

volatile uint8_t *io_port(uint8_t portid){
  io_portfuncs[portid](io.port[portid]);
  return &io.port[portid];
} 


void io_sim_init(io_sim_monitor_t* hooks){
  int i,j;
  for (i=0;i<SIM_PORT_COUNT;i++) {
    //setup default access, then check for overrides
    io_portfuncs[i]=io_default_port_access;

    for (j=0;hooks[j].ddr_addr!=0;j++) {
      if (hooks[j].ddr_addr == &(io.port[i])) {
        io_portfuncs[i]=hooks[j].access_func;
      }
    }
  }
}
