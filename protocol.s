	.file	"protocol.c"
__SREG__ = 0x3f
__SP_H__ = 0x3e
__SP_L__ = 0x3d
__CCP__ = 0x34
__tmp_reg__ = 0
__zero_reg__ = 1
 ;  GNU C (GCC) version 4.5.3 (avr)
 ; 	compiled by GNU C version 4.6.2, GMP version 5.0.2, MPFR version 3.1.0-p3, MPC version 0.9
 ;  GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
 ;  options passed:  -I. -imultilib avr5 -DF_CPU=16000000 protocol.c
 ;  -mmcu=atmega328p -auxbase-strip protocol.s -g -Os -Wall
 ;  -ffunction-sections -fverbose-asm
 ;  options enabled:  -falign-loops -fargument-alias -fauto-inc-dec
 ;  -fbranch-count-reg -fcaller-saves -fcommon -fcprop-registers
 ;  -fcrossjumping -fcse-follow-jumps -fdefer-pop -fearly-inlining
 ;  -feliminate-unused-debug-types -fexpensive-optimizations
 ;  -fforward-propagate -ffunction-cse -ffunction-sections -fgcse -fgcse-lm
 ;  -fguess-branch-probability -fident -fif-conversion -fif-conversion2
 ;  -findirect-inlining -finline -finline-functions
 ;  -finline-functions-called-once -finline-small-functions -fipa-cp
 ;  -fipa-pure-const -fipa-reference -fipa-sra -fira-share-save-slots
 ;  -fira-share-spill-slots -fivopts -fkeep-static-consts
 ;  -fleading-underscore -fmath-errno -fmerge-constants
 ;  -fmerge-debug-strings -fmove-loop-invariants -fomit-frame-pointer
 ;  -foptimize-register-move -foptimize-sibling-calls -fpeephole
 ;  -fpeephole2 -freg-struct-return -fregmove -freorder-blocks
 ;  -freorder-functions -frerun-cse-after-loop
 ;  -fsched-critical-path-heuristic -fsched-dep-count-heuristic
 ;  -fsched-group-heuristic -fsched-interblock -fsched-last-insn-heuristic
 ;  -fsched-rank-heuristic -fsched-spec -fsched-spec-insn-heuristic
 ;  -fsched-stalled-insns-dep -fshow-column -fsigned-zeros
 ;  -fsplit-ivs-in-unroller -fsplit-wide-types -fstrict-aliasing
 ;  -fstrict-overflow -fthread-jumps -ftoplevel-reorder -ftrapping-math
 ;  -ftree-builtin-call-dce -ftree-ccp -ftree-ch -ftree-copy-prop
 ;  -ftree-copyrename -ftree-dce -ftree-dominator-opts -ftree-dse
 ;  -ftree-forwprop -ftree-fre -ftree-loop-im -ftree-loop-ivcanon
 ;  -ftree-loop-optimize -ftree-parallelize-loops= -ftree-phiprop
 ;  -ftree-pre -ftree-pta -ftree-reassoc -ftree-scev-cprop -ftree-sink
 ;  -ftree-slp-vectorize -ftree-sra -ftree-switch-conversion -ftree-ter
 ;  -ftree-vect-loop-version -ftree-vrp -funit-at-a-time -fverbose-asm
 ;  -fzero-initialized-in-bss

	.stabs	"/home/adam/code/grbl/",100,0,2,.Ltext0
	.stabs	"protocol.c",100,0,2,.Ltext0
	.text
.Ltext0:
	.stabs	"gcc2_compiled.",60,0,0,0
	.stabs	"int:t(0,1)=r(0,1);-32768;32767;",128,0,0,0
	.stabs	"char:t(0,2)=r(0,2);0;127;",128,0,0,0
	.stabs	"long int:t(0,3)=@s32;r(0,3);020000000000;017777777777;",128,0,0,0
	.stabs	"unsigned int:t(0,4)=r(0,4);0;0177777;",128,0,0,0
	.stabs	"long unsigned int:t(0,5)=@s32;r(0,5);0;037777777777;",128,0,0,0
	.stabs	"long long int:t(0,6)=@s64;r(0,6);01000000000000000000000;0777777777777777777777;",128,0,0,0
	.stabs	"long long unsigned int:t(0,7)=@s64;r(0,7);0;01777777777777777777777;",128,0,0,0
	.stabs	"short int:t(0,8)=r(0,8);-32768;32767;",128,0,0,0
	.stabs	"short unsigned int:t(0,9)=r(0,9);0;0177777;",128,0,0,0
	.stabs	"signed char:t(0,10)=@s8;r(0,10);-128;127;",128,0,0,0
	.stabs	"unsigned char:t(0,11)=@s8;r(0,11);0;255;",128,0,0,0
	.stabs	"float:t(0,12)=r(0,1);4;0;",128,0,0,0
	.stabs	"double:t(0,13)=r(0,1);4;0;",128,0,0,0
	.stabs	"long double:t(0,14)=r(0,1);4;0;",128,0,0,0
	.stabs	"void:t(0,15)=(0,15)",128,0,0,0
 ;  Compiler executable checksum: a178c2de5985e958d5ea3a6b75766c05

	.stabs	"/usr/lib/gcc/avr/4.5.3/../../../avr/include/avr/io.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/../../../avr/include/avr/sfr_defs.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/../../../avr/include/inttypes.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/include/stdint.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/../../../avr/include/stdint.h",130,0,0,0
	.stabs	"int8_t:t(5,1)=(0,10)",128,0,121,0
	.stabs	"uint8_t:t(5,2)=(0,11)",128,0,122,0
	.stabs	"int16_t:t(5,3)=(0,1)",128,0,123,0
	.stabs	"uint16_t:t(5,4)=(0,4)",128,0,124,0
	.stabs	"int32_t:t(5,5)=(0,3)",128,0,125,0
	.stabs	"uint32_t:t(5,6)=(0,5)",128,0,126,0
	.stabs	"int64_t:t(5,7)=(0,6)",128,0,128,0
	.stabs	"uint64_t:t(5,8)=(0,7)",128,0,129,0
	.stabs	"intptr_t:t(5,9)=(5,3)",128,0,142,0
	.stabs	"uintptr_t:t(5,10)=(5,4)",128,0,147,0
	.stabs	"int_least8_t:t(5,11)=(5,1)",128,0,159,0
	.stabs	"uint_least8_t:t(5,12)=(5,2)",128,0,164,0
	.stabs	"int_least16_t:t(5,13)=(5,3)",128,0,169,0
	.stabs	"uint_least16_t:t(5,14)=(5,4)",128,0,174,0
	.stabs	"int_least32_t:t(5,15)=(5,5)",128,0,179,0
	.stabs	"uint_least32_t:t(5,16)=(5,6)",128,0,184,0
	.stabs	"int_least64_t:t(5,17)=(5,7)",128,0,192,0
	.stabs	"uint_least64_t:t(5,18)=(5,8)",128,0,199,0
	.stabs	"int_fast8_t:t(5,19)=(5,1)",128,0,213,0
	.stabs	"uint_fast8_t:t(5,20)=(5,2)",128,0,218,0
	.stabs	"int_fast16_t:t(5,21)=(5,3)",128,0,223,0
	.stabs	"uint_fast16_t:t(5,22)=(5,4)",128,0,228,0
	.stabs	"int_fast32_t:t(5,23)=(5,5)",128,0,233,0
	.stabs	"uint_fast32_t:t(5,24)=(5,6)",128,0,238,0
	.stabs	"int_fast64_t:t(5,25)=(5,7)",128,0,246,0
	.stabs	"uint_fast64_t:t(5,26)=(5,8)",128,0,253,0
	.stabs	"intmax_t:t(5,27)=(5,7)",128,0,273,0
	.stabs	"uintmax_t:t(5,28)=(5,8)",128,0,278,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"int_farptr_t:t(3,1)=(5,5)",128,0,77,0
	.stabs	"uint_farptr_t:t(3,2)=(5,6)",128,0,81,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/../../../avr/include/avr/fuse.h",130,0,0,0
	.stabs	"__fuse_t:t(6,1)=(6,2)=s3low:(0,11),0,8;high:(0,11),8,8;extended:(0,11),16,8;;",128,0,244,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"gcode.h",130,0,0,0
	.stabs	"nuts_bolts.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/../../../avr/include/string.h",130,0,0,0
	.stabs	"/usr/lib/gcc/avr/4.5.3/include/stddef.h",130,0,0,0
	.stabs	"size_t:t(10,1)=(0,4)",128,0,211,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.stabs	"system_t:t(8,1)=(8,2)=s16abort:(5,2),0,8;state:(5,2),8,8;execute:(8,3)=B(5,2),16,8;position:(8,4)=ar(8,5)=r(8,5);0;0177777;;0;2;(5,5),24,96;auto_start:(5,2),120,8;;",128,0,91,0
	.stabn	162,0,0,0
	.stabs	"parser_state_t:t(7,1)=(7,2)=s53status_code:(5,2),0,8;motion_mode:(5,2),8,8;inverse_feed_rate_mode:(5,2),16,8;inches_mode:(5,2),24,8;absolute_mode:(5,2),32,8;program_flow:(5,2),40,8;spindle_direction:(5,1),48,8;coolant_mode:(5,2),56,8;feed_rate:(0,12),64,32;position:(7,3)=ar(8,5);0;2;(0,12),96,96;tool:(5,2),192,8;plane_axis_0:(5,2),200,8;plane_axis_1:(5,2),208,8;plane_axis_2:(5,2),216,8;coord_select:(5,2),224,8;coord_system:(7,3),232,96;coord_offset:(7,3),328,96;;",128,0,87,0
	.stabn	162,0,0,0
	.stabs	"settings.h",130,0,0,0
	.stabs	"settings_t:t(11,1)=(11,2)=s55steps_per_mm:(7,3),0,96;microsteps:(5,2),96,8;pulse_microseconds:(5,2),104,8;default_feed_rate:(0,12),112,32;default_seek_rate:(0,12),144,32;pulse_invert_mask:(5,2),176,8;dir_invert_mask:(5,2),184,8;mm_per_arc_segment:(0,12),192,32;acceleration:(0,12),224,32;junction_deviation:(0,12),256,32;flags:(5,2),288,8;homing_dir_mask:(5,2),296,8;homing_feed_rate:(0,12),304,32;homing_seek_rate:(0,12),336,32;homing_debounce_delay:(5,4),368,16;homing_pulloff:(0,12),384,32;stepper_idle_lock_time:(5,2),416,8;decimal_places:(5,2),424,8;n_arc_correction:(5,2),432,8;;",128,0,79,0
	.stabn	162,0,0,0
	.stabs	"motion_control.h",130,0,0,0
	.stabs	"planner.h",130,0,0,0
	.stabs	"block_t:t(13,1)=(13,2)=s59direction_bits:(5,2),0,8;steps_x:(5,6),8,32;steps_y:(5,6),40,32;steps_z:(5,6),72,32;step_event_count:(5,5),104,32;nominal_speed:(0,12),136,32;entry_speed:(0,12),168,32;max_entry_speed:(0,12),200,32;millimeters:(0,12),232,32;recalculate_flag:(5,2),264,8;nominal_length_flag:(5,2),272,8;initial_rate:(5,6),280,32;final_rate:(5,6),312,32;rate_delta:(5,5),344,32;accelerate_until:(5,6),376,32;decelerate_after:(5,6),408,32;nominal_rate:(5,6),440,32;;",128,0,55,0
	.stabn	162,0,0,0
	.stabn	162,0,0,0
	.section	.text.protocol_reset_line_buffer,"ax",@progbits
	.stabs	"protocol_reset_line_buffer:f(0,15)",36,0,40,protocol_reset_line_buffer
	.type	protocol_reset_line_buffer, @function
protocol_reset_line_buffer:
	.stabd	46,0,0
	.stabn	68,0,41,.LM0-.LFBB1
.LM0:
.LFBB1:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	.stabn	68,0,42,.LM1-.LFBB1
.LM1:
	sts char_counter,__zero_reg__	 ;  char_counter,
	.stabn	68,0,43,.LM2-.LFBB1
.LM2:
	sts iscomment,__zero_reg__	 ;  iscomment,
/* epilogue start */
	.stabn	68,0,44,.LM3-.LFBB1
.LM3:
	ret
	.size	protocol_reset_line_buffer, .-protocol_reset_line_buffer
.Lscope1:
	.stabs	"",36,0,0,.Lscope1-.LFBB1
	.stabd	78,0,0
	.section	.text.protocol_init,"ax",@progbits
	.stabs	"protocol_init:F(0,15)",36,0,47,protocol_init
.global	protocol_init
	.type	protocol_init, @function
protocol_init:
	.stabd	46,0,0
	.stabn	68,0,48,.LM4-.LFBB2
.LM4:
.LFBB2:
/* prologue: function */
/* frame size = 0 */
/* stack size = 0 */
.L__stack_usage = 0
	.stabn	68,0,49,.LM5-.LFBB2
.LM5:
	call protocol_reset_line_buffer	 ; 
	.stabn	68,0,50,.LM6-.LFBB2
.LM6:
	call report_init_message	 ; 
	.stabn	68,0,52,.LM7-.LFBB2
.LM7:
	cbi 39-32,3	 ; ,,
	.stabn	68,0,53,.LM8-.LFBB2
.LM8:
	sbi 40-32,3	 ; ,,
	.stabn	68,0,54,.LM9-.LFBB2
.LM9:
	ldi r30,lo8(108)	 ;  tmp55,
	ldi r31,hi8(108)	 ;  tmp55,
	ld r24,Z	 ;  D.2765,
	ori r24,lo8(8)	 ;  D.2765,
	st Z,r24	 ; , D.2765
	.stabn	68,0,55,.LM10-.LFBB2
.LM10:
	ldi r30,lo8(104)	 ;  tmp57,
	ldi r31,hi8(104)	 ;  tmp57,
	ld r24,Z	 ;  D.2768,
	ori r24,lo8(2)	 ;  D.2768,
	st Z,r24	 ; , D.2768
/* epilogue start */
	.stabn	68,0,56,.LM11-.LFBB2
.LM11:
	ret
	.size	protocol_init, .-protocol_init
.Lscope2:
	.stabs	"",36,0,0,.Lscope2-.LFBB2
	.stabd	78,0,0
	.section	.text.protocol_execute_startup,"ax",@progbits
	.stabs	"protocol_execute_startup:F(0,15)",36,0,59,protocol_execute_startup
.global	protocol_execute_startup
	.type	protocol_execute_startup, @function
protocol_execute_startup:
	.stabd	46,0,0
	.stabn	68,0,60,.LM12-.LFBB3
.LM12:
.LFBB3:
	push r17	 ; 
/* prologue: function */
/* frame size = 0 */
/* stack size = 1 */
.L__stack_usage = 1
	.stabn	68,0,62,.LM13-.LFBB3
.LM13:
	ldi r17,lo8(0)	 ;  n,
.L6:
	.stabn	68,0,63,.LM14-.LFBB3
.LM14:
	mov r24,r17	 ; , n
	ldi r22,lo8(line)	 ; ,
	ldi r23,hi8(line)	 ; ,
	call settings_read_startup_line	 ; 
	tst r24	 ; 
	brne .L4	 ; ,
	.stabn	68,0,64,.LM15-.LFBB3
.LM15:
	ldi r24,lo8(10)	 ; ,
	rjmp .L8	 ; 
.L4:
	.stabn	68,0,66,.LM16-.LFBB3
.LM16:
	lds r24,line	 ;  line, line
	tst r24	 ;  line
	breq .L5	 ; ,
	.stabn	68,0,67,.LM17-.LFBB3
.LM17:
	ldi r24,lo8(line)	 ; ,
	ldi r25,hi8(line)	 ; ,
	call printString	 ; 
	.stabn	68,0,68,.LM18-.LFBB3
.LM18:
	ldi r24,lo8(line)	 ; ,
	ldi r25,hi8(line)	 ; ,
	call gc_execute_line	 ; 
.L8:
	call report_status_message	 ; 
.L5:
	.stabn	68,0,62,.LM19-.LFBB3
.LM19:
	subi r17,lo8(-(1))	 ;  n,
	cpi r17,lo8(2)	 ;  n,
	brne .L6	 ; ,
/* epilogue start */
	.stabn	68,0,72,.LM20-.LFBB3
.LM20:
	pop r17	 ; 
	ret
	.size	protocol_execute_startup, .-protocol_execute_startup
	.stabs	"n:r(5,2)",64,0,61,17
	.stabn	192,0,0,.LFBB3-.LFBB3
	.stabn	224,0,0,.Lscope3-.LFBB3
.Lscope3:
	.stabs	"",36,0,0,.Lscope3-.LFBB3
	.stabd	78,0,0
	.section	.text.protocol_execute_runtime,"ax",@progbits
	.stabs	"protocol_execute_runtime:F(0,15)",36,0,104,protocol_execute_runtime
.global	protocol_execute_runtime
	.type	protocol_execute_runtime, @function
protocol_execute_runtime:
	.stabd	46,0,0
	.stabn	68,0,105,.LM21-.LFBB4
.LM21:
.LFBB4:
	push r28	 ; 
	push r29	 ; 
/* prologue: function */
/* frame size = 0 */
/* stack size = 2 */
.L__stack_usage = 2
	.stabn	68,0,106,.LM22-.LFBB4
.LM22:
	lds r24,sys+2	 ;  D.2701, sys.execute
	tst r24	 ;  D.2701
	brne .+2	 ; 
	rjmp .L9	 ; 
.LBB2:
	.stabn	68,0,107,.LM23-.LFBB4
.LM23:
	lds r24,sys+2	 ;  rt_exec, sys.execute
	.stabn	68,0,112,.LM24-.LFBB4
.LM24:
	mov r28,r24	 ;  D.2704, rt_exec
	ldi r29,lo8(0)	 ;  D.2704,
	movw r24,r28	 ;  tmp71, D.2704
	andi r24,lo8(96)	 ;  tmp71,
	andi r25,hi8(96)	 ;  tmp71,
	sbiw r24,0	 ;  tmp71
	breq .L11	 ; ,
	.stabn	68,0,113,.LM25-.LFBB4
.LM25:
	ldi r24,lo8(6)	 ;  tmp73,
	sts sys+1,r24	 ;  sys.state, tmp73
	.stabn	68,0,116,.LM26-.LFBB4
.LM26:
	movw r24,r28	 ; , D.2704
	sbrs r24,6	 ; ,
	rjmp .L12	 ; 
	.stabn	68,0,117,.LM27-.LFBB4
.LM27:
	ldi r24,lo8(-1)	 ; ,
	call report_alarm_message	 ; 
	.stabn	68,0,118,.LM28-.LFBB4
.LM28:
	ldi r24,lo8(1)	 ; ,
	call report_feedback_message	 ; 
	.stabn	68,0,119,.LM29-.LFBB4
.LM29:
	lds r24,sys+2	 ;  D.2712, sys.execute
	andi r24,lo8(-17)	 ;  D.2712,
	sts sys+2,r24	 ;  sys.execute, D.2712
.L13:
	.stabn	68,0,124,.LM30-.LFBB4
.LM30:
	lds r24,sys+2	 ;  D.2713, sys.execute
	sbrs r24,4	 ;  D.2713,
	rjmp .L13	 ; 
	rjmp .L14	 ; 
.L12:
	.stabn	68,0,131,.LM31-.LFBB4
.LM31:
	ldi r24,lo8(-2)	 ; ,
	call report_alarm_message	 ; 
.L14:
	.stabn	68,0,133,.LM32-.LFBB4
.LM32:
	lds r24,sys+2	 ;  D.2718, sys.execute
	andi r24,lo8(-97)	 ;  D.2718,
	sts sys+2,r24	 ;  sys.execute, D.2718
.L11:
	.stabn	68,0,137,.LM33-.LFBB4
.LM33:
	sbrs r28,4	 ;  tmp24,
	rjmp .L15	 ; 
	.stabn	68,0,138,.LM34-.LFBB4
.LM34:
	ldi r24,lo8(1)	 ;  tmp87,
	sts sys,r24	 ;  sys.abort, tmp87
	.stabn	68,0,139,.LM35-.LFBB4
.LM35:
	rjmp .L9	 ; 
.L15:
	.stabn	68,0,143,.LM36-.LFBB4
.LM36:
	sbrs r28,0	 ;  tmp24,
	rjmp .L16	 ; 
	.stabn	68,0,144,.LM37-.LFBB4
.LM37:
	call report_realtime_status	 ; 
	.stabn	68,0,145,.LM38-.LFBB4
.LM38:
	lds r24,sys+2	 ;  D.2727, sys.execute
	andi r24,lo8(-2)	 ;  D.2727,
	sts sys+2,r24	 ;  sys.execute, D.2727
.L16:
	.stabn	68,0,149,.LM39-.LFBB4
.LM39:
	sbrs r28,3	 ;  tmp24,
	rjmp .L17	 ; 
	.stabn	68,0,150,.LM40-.LFBB4
.LM40:
	call st_feed_hold	 ; 
	.stabn	68,0,151,.LM41-.LFBB4
.LM41:
	lds r24,sys+2	 ;  D.2732, sys.execute
	andi r24,lo8(-9)	 ;  D.2732,
	sts sys+2,r24	 ;  sys.execute, D.2732
.L17:
	.stabn	68,0,156,.LM42-.LFBB4
.LM42:
	sbrs r28,2	 ;  tmp24,
	rjmp .L18	 ; 
	.stabn	68,0,157,.LM43-.LFBB4
.LM43:
	call st_cycle_reinitialize	 ; 
	.stabn	68,0,158,.LM44-.LFBB4
.LM44:
	lds r24,sys+2	 ;  D.2737, sys.execute
	andi r24,lo8(-5)	 ;  D.2737,
	sts sys+2,r24	 ;  sys.execute, D.2737
.L18:
	.stabn	68,0,161,.LM45-.LFBB4
.LM45:
	sbrs r28,1	 ;  tmp24,
	rjmp .L9	 ; 
	.stabn	68,0,162,.LM46-.LFBB4
.LM46:
	call st_cycle_start	 ; 
	.stabn	68,0,163,.LM47-.LFBB4
.LM47:
	lds r24,settings+36	 ;  settings.flags, settings.flags
	sbrs r24,1	 ;  settings.flags,
	rjmp .L19	 ; 
	.stabn	68,0,164,.LM48-.LFBB4
.LM48:
	ldi r24,lo8(1)	 ;  tmp107,
	sts sys+15,r24	 ;  sys.auto_start, tmp107
.L19:
	.stabn	68,0,166,.LM49-.LFBB4
.LM49:
	lds r24,sys+2	 ;  D.2747, sys.execute
	andi r24,lo8(-3)	 ;  D.2747,
	sts sys+2,r24	 ;  sys.execute, D.2747
.L9:
/* epilogue start */
.LBE2:
	.stabn	68,0,172,.LM50-.LFBB4
.LM50:
	pop r29	 ; 
	pop r28	 ; 
	ret
	.size	protocol_execute_runtime, .-protocol_execute_runtime
	.stabs	"rt_exec:r(5,2)",64,0,107,24
	.stabn	192,0,0,.LBB2-.LFBB4
	.stabn	224,0,0,.LBE2-.LFBB4
.Lscope4:
	.stabs	"",36,0,0,.Lscope4-.LFBB4
	.stabd	78,0,0
	.section	.text.protocol_execute_line,"ax",@progbits
	.stabs	"protocol_execute_line:F(5,2)",36,0,183,protocol_execute_line
	.stabs	"line:P(0,16)=*(0,2)",64,0,183,16
.global	protocol_execute_line
	.type	protocol_execute_line, @function
protocol_execute_line:
	.stabd	46,0,0
	.stabn	68,0,184,.LM51-.LFBB5
.LM51:
.LFBB5:
	push r14	 ; 
	push r15	 ; 
	push r16	 ; 
	push r17	 ; 
	push r29	 ; 
	push r28	 ; 
	in r28,__SP_L__	 ; 
	in r29,__SP_H__	 ; 
	sbiw r28,10	 ; ,
	in __tmp_reg__,__SREG__
	cli
	out __SP_H__,r29	 ; 
	out __SREG__,__tmp_reg__
	out __SP_L__,r28	 ; 
/* prologue: function */
/* frame size = 10 */
/* stack size = 16 */
.L__stack_usage = 16
	movw r16,r24	 ;  line, line
	.stabn	68,0,186,.LM52-.LFBB5
.LM52:
	movw r30,r24	 ; , line
	ld r24,Z	 ;  tmp112,
	cpi r24,lo8(36)	 ;  tmp112,
	breq .+2	 ; 
	rjmp .L22	 ; 
.LBB3:
	.stabn	68,0,188,.LM53-.LFBB5
.LM53:
	ldi r24,lo8(1)	 ;  tmp113,
	std Y+1,r24	 ;  char_counter, tmp113
	.stabn	68,0,191,.LM54-.LFBB5
.LM54:
	ldd r24,Z+1	 ;  tmp114,
	cpi r24,lo8(71)	 ;  tmp114,
	breq .L28	 ; ,
	cpi r24,lo8(72)	 ;  tmp114,
	brge .L32	 ; ,
	cpi r24,lo8(36)	 ;  tmp114,
	breq .L26	 ; ,
	cpi r24,lo8(37)	 ;  tmp114,
	brge .L33	 ; ,
	tst r24	 ;  tmp114
	breq .L24	 ; ,
	cpi r24,lo8(35)	 ;  tmp114,
	breq .+2	 ; 
	rjmp .L43	 ; 
	rjmp .L63	 ; 
.L33:
	cpi r24,lo8(67)	 ;  tmp114,
	breq .+2	 ; 
	rjmp .L43	 ; 
	rjmp .L64	 ; 
.L32:
	cpi r24,lo8(78)	 ;  tmp114,
	brne .+2	 ; 
	rjmp .L30	 ; 
	cpi r24,lo8(88)	 ;  tmp114,
	breq .L31	 ; ,
	cpi r24,lo8(72)	 ;  tmp114,
	breq .+2	 ; 
	rjmp .L43	 ; 
	rjmp .L65	 ; 
.L24:
	.stabn	68,0,192,.LM55-.LFBB5
.LM55:
	call report_grbl_help	 ; 
	rjmp .L54	 ; 
.L26:
	.stabn	68,0,194,.LM56-.LFBB5
.LM56:
	ldi r24,lo8(2)	 ;  tmp116,
	std Y+1,r24	 ;  char_counter, tmp116
	movw r30,r16	 ; , line
	ldd r24,Z+2	 ;  tmp117,
	tst r24	 ;  tmp117
	breq .+2	 ; 
	rjmp .L59	 ; 
	.stabn	68,0,195,.LM57-.LFBB5
.LM57:
	call report_grbl_settings	 ; 
	rjmp .L54	 ; 
.L63:
	.stabn	68,0,198,.LM58-.LFBB5
.LM58:
	ldi r24,lo8(2)	 ;  tmp119,
	std Y+1,r24	 ;  char_counter, tmp119
	movw r30,r16	 ; , line
	ldd r24,Z+2	 ;  tmp120,
	tst r24	 ;  tmp120
	breq .+2	 ; 
	rjmp .L59	 ; 
	.stabn	68,0,199,.LM59-.LFBB5
.LM59:
	call report_gcode_parameters	 ; 
	rjmp .L54	 ; 
.L28:
	.stabn	68,0,202,.LM60-.LFBB5
.LM60:
	ldi r24,lo8(2)	 ;  tmp122,
	std Y+1,r24	 ;  char_counter, tmp122
	movw r30,r16	 ; , line
	ldd r24,Z+2	 ;  tmp123,
	tst r24	 ;  tmp123
	breq .+2	 ; 
	rjmp .L59	 ; 
	.stabn	68,0,203,.LM61-.LFBB5
.LM61:
	call report_gcode_modes	 ; 
	rjmp .L54	 ; 
.L64:
	.stabn	68,0,206,.LM62-.LFBB5
.LM62:
	ldi r24,lo8(2)	 ;  tmp125,
	std Y+1,r24	 ;  char_counter, tmp125
	movw r30,r16	 ; , line
	ldd r24,Z+2	 ;  tmp126,
	tst r24	 ;  tmp126
	breq .+2	 ; 
	rjmp .L59	 ; 
	.stabn	68,0,210,.LM63-.LFBB5
.LM63:
	lds r24,sys+1	 ;  D.2641, sys.state
	cpi r24,lo8(7)	 ;  D.2641,
	brne .L35	 ; ,
	.stabn	68,0,211,.LM64-.LFBB5
.LM64:
	call mc_reset	 ; 
	.stabn	68,0,212,.LM65-.LFBB5
.LM65:
	ldi r24,lo8(5)	 ; ,
	rjmp .L62	 ; 
.L35:
	.stabn	68,0,214,.LM66-.LFBB5
.LM66:
	tst r24	 ;  D.2641
	breq .+2	 ; 
	rjmp .L52	 ; 
	.stabn	68,0,215,.LM67-.LFBB5
.LM67:
	ldi r24,lo8(7)	 ;  tmp131,
	sts sys+1,r24	 ;  sys.state, tmp131
	.stabn	68,0,216,.LM68-.LFBB5
.LM68:
	ldi r24,lo8(4)	 ; ,
.L62:
	call report_feedback_message	 ; 
	rjmp .L54	 ; 
.L31:
	.stabn	68,0,220,.LM69-.LFBB5
.LM69:
	ldi r24,lo8(2)	 ;  tmp133,
	std Y+1,r24	 ;  char_counter, tmp133
	movw r30,r16	 ; , line
	ldd r24,Z+2	 ;  tmp134,
	tst r24	 ;  tmp134
	breq .+2	 ; 
	rjmp .L59	 ; 
	.stabn	68,0,221,.LM70-.LFBB5
.LM70:
	lds r24,sys+1	 ;  sys.state, sys.state
	cpi r24,lo8(6)	 ;  sys.state,
	breq .+2	 ; 
	rjmp .L54	 ; 
	.stabn	68,0,222,.LM71-.LFBB5
.LM71:
	ldi r24,lo8(3)	 ; ,
	call report_feedback_message	 ; 
	.stabn	68,0,223,.LM72-.LFBB5
.LM72:
	sts sys+1,__zero_reg__	 ;  sys.state,
	rjmp .L54	 ; 
.L65:
	.stabn	68,0,230,.LM73-.LFBB5
.LM73:
	lds r24,settings+36	 ;  settings.flags, settings.flags
	sbrs r24,4	 ;  settings.flags,
	rjmp .L51	 ; 
	.stabn	68,0,232,.LM74-.LFBB5
.LM74:
	lds r24,sys+1	 ;  D.2641, sys.state
	tst r24	 ;  D.2641
	breq .L36	 ; ,
	.stabn	68,0,232,.LM75-.LFBB5
.LM75:
	cpi r24,lo8(6)	 ;  D.2641,
	breq .+2	 ; 
	rjmp .L52	 ; 
.L36:
.LBB4:
	.stabn	68,0,234,.LM76-.LFBB5
.LM76:
	ldi r24,lo8(2)	 ;  tmp144,
	std Y+1,r24	 ;  char_counter, tmp144
	movw r30,r16	 ; , line
	ldd r18,Z+2	 ;  D.2629,
	tst r18	 ;  D.2629
	breq .L53	 ; ,
	.stabn	68,0,236,.LM77-.LFBB5
.LM77:
	clr r19	 ;  tmp146
	sbrc r18,7	 ;  tmp146
	com r19	 ;  tmp146
	subi r18,lo8(-(-48))	 ;  tmp146,
	sbci r19,hi8(-(-48))	 ;  tmp146,
	ldi r24,lo8(1)	 ;  tmp147,
	ldi r25,hi8(1)	 ;  tmp147,
	rjmp 2f
1:	lsl r24	 ;  tmp147
	rol r25	 ;  tmp147
2:	dec r18	 ;  tmp146
	brpl 1b
	rjmp .L37	 ; 
.L53:
	.stabn	68,0,233,.LM78-.LFBB5
.LM78:
	ldi r24,lo8(4)	 ;  axis_mask,
.L37:
	.stabn	68,0,238,.LM79-.LFBB5
.LM79:
	call mc_go_home	 ; 
	.stabn	68,0,239,.LM80-.LFBB5
.LM80:
	lds r24,sys	 ;  sys.abort, sys.abort
	tst r24	 ;  sys.abort
	breq .+2	 ; 
	rjmp .L54	 ; 
	.stabn	68,0,239,.LM81-.LFBB5
.LM81:
	call protocol_execute_startup	 ; 
	rjmp .L54	 ; 
.L30:
.LBE4:
	.stabn	68,0,256,.LM82-.LFBB5
.LM82:
	ldi r24,lo8(2)	 ;  tmp153,
	std Y+1,r24	 ;  char_counter, tmp153
	movw r30,r16	 ; , line
	ldd r24,Z+2	 ;  tmp154,
	tst r24	 ;  tmp154
	brne .L55	 ; ,
	.stabn	68,0,258,.LM83-.LFBB5
.LM83:
	movw r22,r16	 ; , line
	call settings_read_startup_line	 ; 
	tst r24	 ; 
	breq .L38	 ; ,
	.stabn	68,0,261,.LM84-.LFBB5
.LM84:
	ldi r24,lo8(0)	 ; ,
	movw r22,r16	 ; , line
	call report_startup_line	 ; 
	rjmp .L39	 ; 
.L38:
	.stabn	68,0,259,.LM85-.LFBB5
.LM85:
	ldi r24,lo8(10)	 ; ,
	call report_status_message	 ; 
.L39:
	.stabn	68,0,258,.LM86-.LFBB5
.LM86:
	ldi r24,lo8(1)	 ; ,
	movw r22,r16	 ; , line
	call settings_read_startup_line	 ; 
	tst r24	 ; 
	breq .L40	 ; ,
	.stabn	68,0,261,.LM87-.LFBB5
.LM87:
	ldi r24,lo8(1)	 ; ,
	movw r22,r16	 ; , line
	call report_startup_line	 ; 
	rjmp .L54	 ; 
.L40:
	.stabn	68,0,259,.LM88-.LFBB5
.LM88:
	ldi r24,lo8(10)	 ; ,
	call report_status_message	 ; 
	rjmp .L54	 ; 
.L43:
	.stabn	68,0,189,.LM89-.LFBB5
.LM89:
	ldi r18,lo8(0)	 ;  helper_var,
	rjmp .L23	 ; 
.L55:
	.stabn	68,0,266,.LM90-.LFBB5
.LM90:
	ldi r18,lo8(1)	 ;  helper_var,
.L23:
	.stabn	68,0,270,.LM91-.LFBB5
.LM91:
	movw r24,r16	 ; , line
	movw r14,r28	 ;  tmp190,
	sec
	adc r14,__zero_reg__	 ;  tmp190
	adc r15,__zero_reg__	 ;  tmp190
	movw r22,r14	 ; , tmp190
	movw r20,r28	 ; ,
	subi r20,lo8(-(2))	 ; ,
	sbci r21,hi8(-(2))	 ; ,
	std Y+10,r18	 ; ,
	call read_float	 ; 
	ldd r18,Y+10	 ; ,
	sbiw r24,0	 ; 
	brne .+2	 ; 
	rjmp .L58	 ; 
	.stabn	68,0,271,.LM92-.LFBB5
.LM92:
	ldd r24,Y+1	 ;  char_counter.4, char_counter
	movw r30,r16	 ;  tmp164, line
	add r30,r24	 ;  tmp164, char_counter.4
	adc r31,__zero_reg__	 ;  tmp164
	ld r25,Z	 ;  D.2629,
	subi r24,lo8(-(1))	 ;  char_counter_lsm.30,
	std Y+1,r24	 ;  char_counter, char_counter_lsm.30
	cpi r25,lo8(61)	 ;  D.2629,
	breq .+2	 ; 
	rjmp .L59	 ; 
	.stabn	68,0,272,.LM93-.LFBB5
.LM93:
	tst r18	 ;  helper_var
	breq .L41	 ; ,
	mov r18,r24	 ;  char_counter_lsm.30, char_counter_lsm.30
	.stabn	68,0,276,.LM94-.LFBB5
.LM94:
	ldi r25,lo8(0)	 ;  char_counter_lsm.30,
.L42:
	.stabn	68,0,276,.LM95-.LFBB5
.LM95:
	mov r30,r18	 ;  char_counter_lsm.30, char_counter_lsm.30
	ldi r31,lo8(0)	 ;  char_counter_lsm.30,
	movw r26,r16	 ;  D.2628, line
	add r26,r30	 ;  D.2628, char_counter_lsm.30
	adc r27,r31	 ;  D.2628, char_counter_lsm.30
	sub r30,r24	 ;  tmp169, char_counter_lsm.30
	sbc r31,r25	 ;  tmp169, char_counter_lsm.30
	add r30,r16	 ;  tmp169, line
	adc r31,r17	 ;  tmp169, line
	ld r19,X	 ;  tmp170,* D.2628
	st Z,r19	 ; , tmp170
	subi r18,lo8(-(1))	 ;  char_counter_lsm.30,
	.stabn	68,0,277,.LM96-.LFBB5
.LM96:
	ld r19,X	 ;  tmp171,* D.2628
	tst r19	 ;  tmp171
	brne .L42	 ; ,
	.stabn	68,0,277,.LM97-.LFBB5
.LM97:
	std Y+1,r18	 ;  char_counter, char_counter_lsm.30
	.stabn	68,0,279,.LM98-.LFBB5
.LM98:
	movw r24,r16	 ; , line
	call gc_execute_line	 ; 
	mov r18,r24	 ;  helper_var,
	.stabn	68,0,280,.LM99-.LFBB5
.LM99:
	tst r24	 ;  helper_var
	brne .L34	 ; ,
	.stabn	68,0,282,.LM100-.LFBB5
.LM100:
	ldd r22,Y+2	 ;  parameter, parameter
	ldd r23,Y+3	 ;  parameter, parameter
	ldd r24,Y+4	 ;  parameter, parameter
	ldd r25,Y+5	 ;  parameter, parameter
	std Y+10,r18	 ; ,
	call trunc	 ; 
	call __fixunssfsi	 ; 
	mov r24,r22	 ;  tmp192,
	.stabn	68,0,283,.LM101-.LFBB5
.LM101:
	movw r22,r16	 ; , line
	call settings_store_startup_line	 ; 
	ldd r18,Y+10	 ; ,
	rjmp .L34	 ; 
.L41:
	.stabn	68,0,286,.LM102-.LFBB5
.LM102:
	movw r24,r16	 ; , line
	movw r22,r14	 ; , tmp190
	movw r20,r28	 ; ,
	subi r20,lo8(-(6))	 ; ,
	sbci r21,hi8(-(6))	 ; ,
	call read_float	 ; 
	sbiw r24,0	 ; 
	breq .L58	 ; ,
	.stabn	68,0,287,.LM103-.LFBB5
.LM103:
	ldd r24,Y+1	 ;  char_counter, char_counter
	movw r30,r16	 ;  tmp182, line
	add r30,r24	 ;  tmp182, char_counter
	adc r31,__zero_reg__	 ;  tmp182
	ld r24,Z	 ;  tmp183,
	tst r24	 ;  tmp183
	brne .L59	 ; ,
	.stabn	68,0,288,.LM104-.LFBB5
.LM104:
	ldd r22,Y+2	 ; , parameter
	ldd r23,Y+3	 ; , parameter
	ldd r24,Y+4	 ; , parameter
	ldd r25,Y+5	 ; , parameter
	call __fixsfsi	 ; 
	movw r26,r24	 ;  tmp186,
	movw r24,r22	 ;  tmp186,
	ldd r20,Y+6	 ;  value, value
	ldd r21,Y+7	 ;  value, value
	ldd r22,Y+8	 ;  value, value
	ldd r23,Y+9	 ;  value, value
	call settings_store_global_setting	 ; 
	rjmp .L61	 ; 
.L22:
.LBE3:
	.stabn	68,0,294,.LM105-.LFBB5
.LM105:
	movw r24,r16	 ; , line
	call gc_execute_line	 ; 
.L61:
	mov r18,r24	 ;  helper_var,
	rjmp .L34	 ; 
.L51:
.LBB6:
	.stabn	68,0,241,.LM106-.LFBB5
.LM106:
	ldi r18,lo8(7)	 ;  helper_var,
	rjmp .L34	 ; 
.L52:
	.stabn	68,0,240,.LM107-.LFBB5
.LM107:
	ldi r18,lo8(11)	 ;  helper_var,
	rjmp .L34	 ; 
.L54:
.LBB5:
	.stabn	68,0,291,.LM108-.LFBB5
.LM108:
	ldi r18,lo8(0)	 ;  helper_var,
	rjmp .L34	 ; 
.L58:
.LBE5:
	.stabn	68,0,286,.LM109-.LFBB5
.LM109:
	ldi r18,lo8(1)	 ;  helper_var,
	rjmp .L34	 ; 
.L59:
	.stabn	68,0,287,.LM110-.LFBB5
.LM110:
	ldi r18,lo8(3)	 ;  helper_var,
.L34:
.LBE6:
	.stabn	68,0,296,.LM111-.LFBB5
.LM111:
	mov r24,r18	 ; , helper_var
/* epilogue start */
	adiw r28,10	 ; ,
	in __tmp_reg__,__SREG__
	cli
	out __SP_H__,r29	 ; 
	out __SREG__,__tmp_reg__
	out __SP_L__,r28	 ; 
	pop r28	 ; 
	pop r29	 ; 
	pop r17	 ; 
	pop r16	 ; 
	pop r15	 ; 
	pop r14	 ; 
	ret
	.size	protocol_execute_line, .-protocol_execute_line
	.stabs	"char_counter:(5,2)",128,0,188,1
	.stabs	"parameter:(0,12)",128,0,190,2
	.stabs	"value:(0,12)",128,0,190,6
	.stabn	192,0,0,.LBB3-.LFBB5
	.stabs	"axis_mask:r(5,2)",64,0,233,24
	.stabn	192,0,0,.LBB4-.LFBB5
	.stabn	224,0,0,.LBE4-.LFBB5
	.stabs	"axis_mask:r(5,2)",64,0,233,24
	.stabn	192,0,0,.LBB5-.LFBB5
	.stabn	224,0,0,.LBE5-.LFBB5
	.stabn	224,0,0,.LBE3-.LFBB5
	.stabs	"char_counter:(5,2)",128,0,188,1
	.stabs	"parameter:(0,12)",128,0,190,2
	.stabs	"value:(0,12)",128,0,190,6
	.stabn	192,0,0,.LBB6-.LFBB5
	.stabn	224,0,0,.LBE6-.LFBB5
.Lscope5:
	.stabs	"",36,0,0,.Lscope5-.LFBB5
	.stabd	78,0,0
	.section	.text.protocol_process,"ax",@progbits
	.stabs	"protocol_process:F(0,15)",36,0,301,protocol_process
.global	protocol_process
	.type	protocol_process, @function
protocol_process:
	.stabd	46,0,0
	.stabn	68,0,302,.LM112-.LFBB6
.LM112:
.LFBB6:
	push r17	 ; 
/* prologue: function */
/* frame size = 0 */
/* stack size = 1 */
.L__stack_usage = 1
	.stabn	68,0,336,.LM113-.LFBB6
.LM113:
	ldi r17,lo8(1)	 ;  tmp86,
	.stabn	68,0,304,.LM114-.LFBB6
.LM114:
	rjmp .L81	 ; 
.L78:
	.stabn	68,0,305,.LM115-.LFBB6
.LM115:
	cpi r24,lo8(10)	 ;  c,
	breq .L68	 ; ,
	.stabn	68,0,305,.LM116-.LFBB6
.LM116:
	cpi r24,lo8(13)	 ;  c,
	brne .L69	 ; ,
.L68:
	.stabn	68,0,310,.LM117-.LFBB6
.LM117:
	call protocol_execute_runtime	 ; 
	.stabn	68,0,311,.LM118-.LFBB6
.LM118:
	lds r24,sys	 ;  sys.abort, sys.abort
	tst r24	 ;  sys.abort
	breq .+2	 ; 
	rjmp .L66	 ; 
	.stabn	68,0,313,.LM119-.LFBB6
.LM119:
	lds r30,char_counter	 ;  char_counter.0, char_counter
	tst r30	 ;  char_counter.0
	breq .L71	 ; ,
	.stabn	68,0,314,.LM120-.LFBB6
.LM120:
	ldi r31,lo8(0)	 ; ,
	subi r30,lo8(-(line))	 ;  tmp61,
	sbci r31,hi8(-(line))	 ;  tmp61,
	st Z,__zero_reg__	 ;  line,
	.stabn	68,0,315,.LM121-.LFBB6
.LM121:
	ldi r24,lo8(line)	 ; ,
	ldi r25,hi8(line)	 ; ,
	call protocol_execute_line	 ; 
	rjmp .L85	 ; 
.L71:
	.stabn	68,0,318,.LM122-.LFBB6
.LM122:
	ldi r24,lo8(0)	 ; ,
	rjmp .L85	 ; 
.L69:
	.stabn	68,0,323,.LM123-.LFBB6
.LM123:
	lds r25,iscomment	 ;  iscomment, iscomment
	tst r25	 ;  iscomment
	breq .L74	 ; ,
	.stabn	68,0,325,.LM124-.LFBB6
.LM124:
	cpi r24,lo8(41)	 ;  c,
	brne .L81	 ; ,
	.stabn	68,0,327,.LM125-.LFBB6
.LM125:
	sts iscomment,__zero_reg__	 ;  iscomment,
	rjmp .L81	 ; 
.L74:
	.stabn	68,0,330,.LM126-.LFBB6
.LM126:
	cpi r24,lo8(33)	 ;  c,
	brlo .L81	 ; ,
	.stabn	68,0,332,.LM127-.LFBB6
.LM127:
	cpi r24,lo8(47)	 ;  c,
	breq .L81	 ; ,
	.stabn	68,0,334,.LM128-.LFBB6
.LM128:
	cpi r24,lo8(40)	 ;  c,
	brne .L75	 ; ,
	.stabn	68,0,336,.LM129-.LFBB6
.LM129:
	sts iscomment,r17	 ;  iscomment, tmp86
	rjmp .L81	 ; 
.L75:
	.stabn	68,0,337,.LM130-.LFBB6
.LM130:
	lds r25,char_counter	 ;  char_counter.0, char_counter
	cpi r25,lo8(69)	 ;  char_counter.0,
	brlo .L76	 ; ,
	.stabn	68,0,339,.LM131-.LFBB6
.LM131:
	ldi r24,lo8(13)	 ; ,
.L85:
	call report_status_message	 ; 
	.stabn	68,0,340,.LM132-.LFBB6
.LM132:
	call protocol_reset_line_buffer	 ; 
	rjmp .L81	 ; 
.L76:
	.stabn	68,0,341,.LM133-.LFBB6
.LM133:
	mov r20,r24	 ;  tmp71, c
	subi r20,lo8(-(-97))	 ;  tmp71,
	mov r18,r25	 ;  char_counter.0, char_counter.0
	ldi r19,lo8(0)	 ;  char_counter.0,
	subi r25,lo8(-(1))	 ;  tmp83,
	cpi r20,lo8(26)	 ;  tmp71,
	brsh .L77	 ; ,
	.stabn	68,0,342,.LM134-.LFBB6
.LM134:
	subi r18,lo8(-(line))	 ;  tmp74,
	sbci r19,hi8(-(line))	 ;  tmp74,
	subi r24,lo8(-(-32))	 ;  tmp75,
	rjmp .L84	 ; 
.L77:
	.stabn	68,0,344,.LM135-.LFBB6
.LM135:
	subi r18,lo8(-(line))	 ;  tmp79,
	sbci r19,hi8(-(line))	 ;  tmp79,
.L84:
	movw r30,r18	 ; , tmp79
	st Z,r24	 ;  line, c
	sts char_counter,r25	 ;  char_counter, tmp83
.L81:
	.stabn	68,0,304,.LM136-.LFBB6
.LM136:
	call serial_read	 ; 
	cpi r24,lo8(-1)	 ;  c,
	breq .+2	 ; 
	rjmp .L78	 ; 
.L66:
/* epilogue start */
	.stabn	68,0,349,.LM137-.LFBB6
.LM137:
	pop r17	 ; 
	ret
	.size	protocol_process, .-protocol_process
	.stabs	"c:r(5,2)",64,0,303,24
	.stabn	192,0,0,.LFBB6-.LFBB6
	.stabn	224,0,0,.Lscope6-.LFBB6
.Lscope6:
	.stabs	"",36,0,0,.Lscope6-.LFBB6
	.stabd	78,0,0
	.lcomm char_counter,1
	.lcomm line,70
	.lcomm iscomment,1
	.stabs	"line:S(0,17)=ar(8,5);0;69;(0,2)",40,0,35,line
	.stabs	"char_counter:S(5,2)",40,0,36,char_counter
	.stabs	"iscomment:S(5,2)",40,0,37,iscomment
	.text
	.stabs	"",100,0,0,.Letext0
.Letext0:
.global __do_clear_bss
