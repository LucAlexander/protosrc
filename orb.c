#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "orb.h"
#include "hashmap.h"

#pragma GCC diagnostic ignored "-Wsequence-point"

MAP_IMPL(OPCODE)
MAP_IMPL(REGISTER)
MAP_IMPL(REG_PARTITION)
MAP_IMPL(block_scope)

#define SHORT(lit) (lit&0xFF00)>>8, lit&0xFF
#define NOP_                   NOP, 0,    0, 0
#define LDS_(dest, lit)        LDS, dest, SHORT(lit)
#define LDA_(dest, addr, off)  LDA, dest, addr, off
#define LDI_(dest, addr, lit)  LDI, dest, addr, lit
#define STS_(src, lit)         STS, src,  SHORT(lit)
#define STA_(src, addr, off)   STA, src,  addr, off
#define STB_(src, addr, lit)   STB, src,  addr, lit
#define MOV_(dst, src)         MOV, dst,  src, 0
#define SWP_(dst, src)         SWP, dst,  src, 0
#define ADS_(dst, right)       ADS, dst,  SHORT(right)
#define SUS_(dst, right)       SUS, dst,  SHORT(right)
#define MUS_(dst, right)       MUS, dst,  SHORT(right)
#define DIS_(dst, right)       DIS, dst,  SHORT(right)
#define MOS_(dst, right)       MOS, dst,  SHORT(right)
#define ANS_(dst, right)       ANS, dst,  SHORT(right)
#define ORS_(dst, right)       ORS, dst,  SHORT(right)
#define SLS_(dst, right)       SLS, dst,  SHORT(right)
#define SRS_(dst, right)       SRS, dst,  SHORT(right)
#define XRS_(dst, right)       XRS, dst,  SHORT(right)
#define ADI_(dst, right)       ADI, dst,  right, 0
#define SUI_(dst, right)       SUI, dst,  right, 0
#define MUI_(dst, right)       MUI, dst,  right, 0
#define DII_(dst, right)       DII, dst,  right, 0
#define MOI_(dst, right)       MOI, dst,  right, 0
#define ANI_(dst, right)       ANI, dst,  right, 0
#define ORI_(dst, right)       ORI, dst,  right, 0
#define SLI_(dst, right)       SLI, dst,  right, 0
#define SRI_(dst, right)       SRI, dst,  right, 0
#define XRI_(dst, right)       XRI, dst,  right, 0
#define ADD_(dst, left, right) ADD, dst,  left, right
#define SUB_(dst, left, right) SUB, dst,  left, right
#define MUL_(dst, left, right) MUL, dst,  left, right
#define DIV_(dst, left, right) DIV, dst,  left, right
#define MOD_(dst, left, right) MOD, dst,  left, right
#define AND_(dst, left, right) AND, dst,  left, right
#define OR_(dst, left, right)  OR,  dst,  left, right
#define SHL_(dst, left, right) SHL, dst,  left, right
#define SHR_(dst, left, right) SHR, dst,  left, right
#define XOR_(dst, left, right) XOR, dst,  left, right
#define INV_(dst, src)         INV, dst,  src, 0
#define COM_(dst, src)         COM, dst,  src, 0
#define INI_(dst)              INI, dst,  0, 0
#define COI_(dst)              COI, dst,  0, 0
#define CMP_(left, right)      CMP, left, right, 0
#define CMS_(left, right)      CMS, left, SHORT(right)
#define RET_(tar)              RET, tar,  0, 0
#define REI_                   REI, 0,    0, 0
#define RES_(tar)              RES, SHORT(tar), 0
#define ESC_                   ESC, 0,    0, 0
#define CAL_                   CAL, 0,    0, 0
#define PSH_(tar)              PSH, tar,  0, 0
#define PSS_(tar)              PSS, SHORT(tar), 0
#define POP_(dst)              POP, dst,  0, 0
/* MODES
 * 0: jump by relative register stored offset
 * 1: jump by relative short literal offset
 * 2: jump to register stored address
*/
#define BNC_(mode, addr)       BNC, mode, SHORT(addr)
#define BNE_(mode, addr)       BNE, mode, SHORT(addr)
#define BEQ_(mode, addr)       BEQ, mode, SHORT(addr)
#define BLT_(mode, addr)       BLT, mode, SHORT(addr)
#define BGT_(mode, addr)       BGT, mode, SHORT(addr)
#define BLE_(mode, addr)       BLE, mode, SHORT(addr)
#define BGE_(mode, addr)       BGE, mode, SHORT(addr)
#define JMP_(mode, off)        JMP, mode, SHORT(off)
#define JNE_(mode, off)        JNE, mode, SHORT(off)
#define JEQ_(mode, off)        JEQ, mode, SHORT(off)
#define JLT_(mode, off)        JLT, mode, SHORT(off)
#define JGT_(mode, off)        JGT, mode, SHORT(off)
#define JLE_(mode, off)        JLE, mode, SHORT(off)
#define JGE_(mode, off)        JGE, mode, SHORT(off)
#define INT_(b)                INT, b,    0, 0
#define INR_(en)               INR, en,   0, 0

#define REG(p, r) (p<<4 | r)

#define INSTRUCTION_WIDTH 0x4
#define PROGRAM_START 0x200
#define MEMORY_SIZE 0x100000

word reg[REGISTER_COUNT];
uint16_t* quar[REGISTER_COUNT*4];
uint32_t* half[REGISTER_COUNT];
byte* lo[REGISTER_COUNT];
byte* hi[REGISTER_COUNT];
byte mem[MEMORY_SIZE];

#define SHOW_REG(r)\
	printf("                                   %s: %016lx (%lu) (%ld)\n\033[0m\033[1m", #r, reg[r], reg[r], reg[r]);

void show_registers(){
	printf("                                   \033[4mCPU Registers\033[0m\n");
	printf("\033[1;32m");
	SHOW_REG(IP);
	printf("\033[1;33m");
	SHOW_REG(SP);
	SHOW_REG(CR);
	printf("\033[1;31m");
	SHOW_REG(FP);
	SHOW_REG(SR);
	SHOW_REG(LR);
	printf("\n");
	SHOW_REG(R0);
	SHOW_REG(R1);
	SHOW_REG(R2);
	SHOW_REG(R3);
	SHOW_REG(R4);
	SHOW_REG(R5);
	SHOW_REG(R6);
	SHOW_REG(R7);
	SHOW_REG(R8);
	SHOW_REG(R9);
	printf("\n");
}

void show_mem(){
	printf("\033[4mProgram / Stack\033[0m\n");
	printf("\033[1;32m");
	for (byte i = 0;i<INSTRUCTION_WIDTH*8;){
		word address = PROGRAM_START+(INSTRUCTION_WIDTH*reg[IP]);
		printf("%016lx | ", address+i);
		byte m = i+INSTRUCTION_WIDTH;
		for (;i<m;++i){
			printf("%02x \033[0m\033[1m", mem[address+i]);
		}
		printf("\033[0m\033[1m\n");
	}
	printf("...              | ...\n");
	word left = reg[SP];
	while ((left--) % 4 != 1){}
	word bottom = MEMORY_SIZE;
	if (reg[SP] < bottom-INSTRUCTION_WIDTH*24){
		bottom = reg[SP] + INSTRUCTION_WIDTH*24;
	}
	for (word i = left;i<bottom;){
		word m = i+INSTRUCTION_WIDTH;
		printf("%016lx | ", i);
		for (;i<m;++i){
			if (i == reg[SP]){
				printf("\033[1;33m");
			}
			else if (i == reg[FP]){
				printf("\033[1;31m");
			}
			printf("%02x \033[0m\033[1m", mem[i]);
		}
		printf("\033[0m\033[1m\n");
	}
}

void show_machine(){
	printf("\033[2J");
	printf("\033[H\033[1m");
	show_registers();
	printf("\033[H\033[1m");
	show_mem();
}
     
#define NEXT (mem[++ip])
#define SHORT_LITERAL ((NEXT<<8)+(NEXT))
#define LOAD_REG(b, v)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		reg[r] = v;\
		break;\
	case HALF:\
		*half[r] = v;\
		break;\
	case LO:\
		*lo[r] = v;\
		break;\
	case HI:\
		*hi[r] = v;\
		break;\
	default:\
		*quar[(r*4)+partition] = v;\
		break;\
	}\
}

#define LOAD_REG_ADDR(b, v)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		reg[r] = *(word*)(&mem[v]);\
		break;\
	case HALF:\
		*half[r] = *(uint32_t*)(&mem[v]);\
		break;\
	case LO:\
		*lo[r] = mem[v];\
		break;\
	case HI:\
		*hi[r] = mem[v];\
		break;\
	default:\
		*quar[(r*4)+partition] = *(uint16_t*)(&mem[v]);\
		break;\
	}\
}

#define ACCESS_REG(v, b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		v = reg[r];\
		break;\
	case HALF:\
		v = *half[r];\
		break;\
	case LO:\
		v = *lo[r];\
		break;\
	case HI:\
		v = *hi[r];\
		break;\
	default:\
		v = *quar[(r*4)+partition];\
		break;\
	}\
}

#define STORE_REG(a, b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		*(word*)(&mem[a]) = reg[r];\
		break;\
	case HALF:\
		*(uint32_t*)(&mem[a]) = *half[r];\
		break;\
	case LO:\
		*(byte*)(&mem[a]) = *lo[r];\
		break;\
	case HI:\
		*(byte*)(&mem[a]) = *hi[r];\
		break;\
	default:\
		*(uint16_t*)(&mem[a]) = *quar[(r*4)+partition];\
		break;\
	}\
}

#define PUSH_REG(b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch(partition){\
	case FULL:\
		reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mem[reg[SP]]) = reg[r];\
		reg[SP] -= 1;\
		break;\
	case HALF:\
		reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mem[reg[SP]]) = (word)(*half[r]);\
		reg[SP] -= 1;\
		break;\
	case LO:\
		reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mem[reg[SP]]) = (word)(*lo[r]);\
		reg[SP] -= 1;\
		break;\
	case HI:\
		reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mem[reg[SP]]) = (word)(*hi[r]);\
		reg[SP] -= 1;\
		break;\
	default:\
		reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mem[reg[SP]]) = (word)(*quar[(r*4)+partition]);\
		reg[SP] -= 1;\
		break;\
	}\
}

#define POP_REG(b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch(partition){\
	case FULL:\
		reg[SP] += 1;\
		reg[r] = *(word*)(&mem[reg[SP]]);\
		reg[SP] += sizeof(word)-1;\
		break;\
	case HALF:\
		reg[SP] += 1;\
		*half[r] = *(word*)(&mem[reg[SP]]);\
		reg[SP] += sizeof(word)-1;\
		break;\
	case LO:\
		reg[SP] += 1;\
		*lo[r] = *(word*)(&mem[reg[SP]]);\
		reg[SP] += sizeof(word)-1;\
		break;\
	case HI:\
		reg[SP] += 1;\
		*hi[r] = *(word*)(&mem[reg[SP]]);\
		reg[SP] += sizeof(word)-1;\
		break;\
	default:\
		reg[SP] += 1;\
		*quar[(r*4)+partition] = *(word*)(&mem[reg[SP]]);\
		reg[SP] += sizeof(word)-1;\
		break;\
	}\
}

#define ALU_S(operator)\
	byte dst = NEXT;\
	uint16_t right = SHORT_LITERAL;\
	word left; ACCESS_REG(left, dst);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += 1;

#define ALU_I(operator)\
	byte dst = NEXT;\
	byte src = NEXT;\
	word left; ACCESS_REG(left, dst);\
	word right; ACCESS_REG(right, src);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += 1;

#define ALU(operator)\
	byte dst = NEXT;\
	byte src_a = NEXT;\
	byte src_b = NEXT;\
	word left; ACCESS_REG(left, src_a);\
	word right; ACCESS_REG(right, src_b);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += 1;

#define ALU_UI(operator)\
	byte tar = NEXT;\
	word val; ACCESS_REG(val, tar);\
	LOAD_REG(tar, operator val);\
	reg[IP] += 1;

#define ALU_U(operator)\
	byte tar = NEXT;\
	byte src = NEXT;\
	word val; ACCESS_REG(val, src);\
	LOAD_REG(tar, operator val);\
	reg[IP] += 1;

#define COMPARE_FLAGS(a, b)\
	if (a<b){\
		reg[SR] |= CARRY; \
		reg[SR] &= ~ZERO;\
	}\
	else if (a==b){\
		reg[SR] |= ZERO; \
		reg[SR] &= ~CARRY;\
	}\
	else{\
		reg[SR] &= ~ZERO; \
		reg[SR] &= ~CARRY;\
	}

#define BRANCH_LINK\
	PUSH_REG(REG(FULL, IP));\
	PUSH_REG(REG(FULL, FP));\
	reg[FP] = reg[SP];\
	ip += 1;\
	byte adr = NEXT;\
	word val; ACCESS_REG(val, adr);\
	reg[IP] += *(int64_t*)(&val);

#define BRANCH_LINK_TO\
	PUSH_REG(REG(FULL, IP));\
	PUSH_REG(REG(FULL, FP));\
	reg[FP] = reg[SP];\
	reg[IP] = reg[LR];

#define BRANCH_JUMP\
	PUSH_REG(REG(FULL, IP));\
	PUSH_REG(REG(FULL, FP));\
	reg[FP] = reg[SP];\
	int16_t offset = SHORT_LITERAL;\
	reg[IP] += offset;

#define JUMP_REG\
	ip += 1;\
	byte adr = NEXT;\
	word val; ACCESS_REG(val, adr);\
	reg[IP] += *(int64_t*)(&val);

#define JUMP_REG_TO\
	reg[IP] = reg[LR];\

#define JUMP\
	int16_t offset = SHORT_LITERAL;\
	reg[IP] += offset;

void interpret(){
	while (1){
		show_machine();
		getc(stdin);
		word ip = PROGRAM_START+(reg[IP]*INSTRUCTION_WIDTH);
		byte op = mem[ip];
		switch (op){
		case NOP: { reg[IP] += 1; } break;
		case LDS: { byte b = NEXT; LOAD_REG(b, SHORT_LITERAL); reg[IP] += 1; } break;
		case LDA: {
				byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				word o; ACCESS_REG(o, off);
				LOAD_REG_ADDR(dst, a+o);
				reg[IP] += 1;
			} break;
		case LDI: {
				byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				LOAD_REG_ADDR(dst, a+off);
				reg[IP] += 1;
			} break;
		case STS: { byte src = NEXT; uint16_t adr = SHORT_LITERAL; STORE_REG(adr, src); reg[IP] += 1; } break;
		case STA: {
				byte src = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				word o; ACCESS_REG(o, off);
				STORE_REG(a+o, src);
				reg[IP] += 1;
			} break;
		case STB: {
				byte src = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				STORE_REG(a+off, src);
				reg[IP] += 1;
			} break;
		case MOV: {
				byte dst = NEXT; byte src = NEXT;
				word s; ACCESS_REG(s, src);
				LOAD_REG(dst, s);
				reg[IP] += 1;
			} break;
		case SWP: {
				byte a = NEXT; byte b = NEXT;
				word s; ACCESS_REG(s, a);
				word d; ACCESS_REG(d, b);
				LOAD_REG(a, d); LOAD_REG(b, s);
				reg[IP] += 1;
			} break;
		case ADS: { ALU_S(+); } break;
		case SUS: { ALU_S(-); } break;
		case MUS: { ALU_S(*); } break;
		case DIS: { ALU_S(/); } break;
		case MOS: { ALU_S(%); } break;
		case ANS: { ALU_S(&); } break;
		case ORS: { ALU_S(|); } break;
		case SLS: { ALU_S(<<); } break;
		case SRS: { ALU_S(>>); } break;
		case XRS: { ALU_S(^); } break;
		case ADI: { ALU_I(+); } break;
		case SUI: { ALU_I(-); } break;
		case MUI: { ALU_I(*); } break;
		case DII: { ALU_I(/); } break;
		case MOI: { ALU_I(%); } break;
		case ANI: { ALU_I(&); } break;
		case ORI: { ALU_I(|); } break;
		case SLI: { ALU_I(<<); } break;
		case SRI: { ALU_I(>>); } break;
		case XRI: { ALU_I(^); } break;
		case ADD: { ALU(+); } break;
		case SUB: { ALU(-); } break;
		case MUL: { ALU(*); } break;
		case DIV: { ALU(/); } break;
		case MOD: { ALU(%); } break;
		case AND: { ALU(&); } break;
		case OR: { ALU(|); } break;
		case SHL: { ALU(<<); } break;
		case SHR: { ALU(>>); } break;
		case XOR: { ALU(^); } break;
		case INV: { ALU_U(!); } break;
		case COM: { ALU_U(~); } break;
		case INI: { ALU_UI(!); } break;
		case COI: { ALU_UI(~); } break;
		case CMP: {
				byte a = NEXT; byte b = NEXT;
				word left; ACCESS_REG(left, a);
				word right; ACCESS_REG(right, b);
				COMPARE_FLAGS(left, right);
				reg[IP] += 1;
			} break;
		case CMS: {
				byte a = NEXT; uint16_t right = SHORT_LITERAL;
				word left; ACCESS_REG(left, a);
				COMPARE_FLAGS(left, right);
				reg[IP] += 1;
			} break;
		case RET: {
				byte tar = NEXT;
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP));
				POP_REG(REG(FULL,IP));
				reg[SP] = reg[CR];
				POP_REG(REG(FULL,CR));
				PUSH_REG(tar);
				reg[IP] += 1;
			} break;
		case REI: {
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP));
				POP_REG(REG(FULL,IP));
				reg[SP] = reg[CR];
				POP_REG(REG(FULL,CR));
				reg[IP] += 1;
			} break;
		case RES: {
				uint16_t tar = SHORT_LITERAL;
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP))
				POP_REG(REG(FULL,IP))
				reg[SP] = reg[CR];
				POP_REG(REG(FULL,CR));
				reg[SP] -= (sizeof(word)-1);
				*(word*)(&mem[reg[SP]]) = tar;
				reg[SP] -= 1;
				reg[IP] += 1;
			} break;
		case ESC: {
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP))
				reg[SP] = reg[CR];
				POP_REG(REG(FULL,CR));
				reg[IP] += 1;
			} break;
		case CAL: {
				PUSH_REG(REG(FULL,CR));
				reg[CR] = reg[SP];
				reg[IP] += 1;
			} break;
		case PSH: { byte tar = NEXT; PUSH_REG(tar); reg[IP] += 1; } break;
		case PSS: {
				uint16_t lit = SHORT_LITERAL;
				reg[SP] -= (sizeof(word)-1);
				*(word*)(&mem[reg[SP]]) = lit;
				reg[SP] -= 1;
				reg[IP] += 1;
			} break;
		case POP: { byte tar = NEXT; POP_REG(tar); reg[IP] += 1; } break;
		case BNC: {
				byte mode = NEXT;
				if (mode == 0) { BRANCH_LINK; }
				else if (mode == 1) { BRANCH_JUMP; }
				else { BRANCH_LINK_TO; }
			} break;
		case BNE: {
				if (((reg[SR] & ZERO) == 0) || ((reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case BEQ: {
				if (((reg[SR] & ZERO) != 0) && ((reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case BLT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case BGT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case BLE: {
				if ((reg[SR] & ZERO) != (reg[SR] & CARRY)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case BGE: {
				if ((reg[SR] & CARRY) == 0){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case JMP: {
				byte mode = NEXT;
				if (mode == 0) { JUMP_REG; } else if (mode == 1) { JUMP; } else { JUMP_REG_TO; }
			} break;
		case JNE: {
				if (((reg[SR] & ZERO) == 0) || ((reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case JEQ: {
				if (((reg[SR] & ZERO) != 0) && ((reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case JLT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case JGT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case JLE: {
				if ((reg[SR] & ZERO) != (reg[SR] & CARRY)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case JGE: {
				if ((reg[SR] & CARRY) == 0){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { reg[IP] += 1; }
			} break;
		case INT: { reg[IP] += 1; } break;
		case INR: { reg[IP] += 1; } break;
		default:
			printf("Unknown upcode\n");
			return;
		}
	}
}

#define ASSERT_LOCAL(b, ...)\
	if (!(b)){\
		snprintf(comp->err, ERROR_BUFFER, "Failed local assertion " #b "\n" __VA_ARGS__);\
		return 0;\
	}

#define ASSERT_ERR(v)\
	if (*comp->err != 0){\
		return v;\
	}

void setup_opcode_map(OPCODE_map* opmap){
	OPCODE* ops = pool_request(opmap->mem, sizeof(OPCODE)*OPCODE_COUNT);
	for (OPCODE i = 0;i<OPCODE_COUNT;++i){
		ops[i] = i;
	}
	OPCODE_map_insert(opmap, "NOP", ops++);
	OPCODE_map_insert(opmap, "LDS", ops++);
	OPCODE_map_insert(opmap, "LDA", ops++);
	OPCODE_map_insert(opmap, "LDI", ops++);
	OPCODE_map_insert(opmap, "STS", ops++);
	OPCODE_map_insert(opmap, "STA", ops++);
	OPCODE_map_insert(opmap, "STB", ops++);
	OPCODE_map_insert(opmap, "MOV", ops++);
	OPCODE_map_insert(opmap, "SWP", ops++);
	OPCODE_map_insert(opmap, "ADS", ops++);
	OPCODE_map_insert(opmap, "SUS", ops++);
	OPCODE_map_insert(opmap, "MUS", ops++);
	OPCODE_map_insert(opmap, "DIS", ops++);
	OPCODE_map_insert(opmap, "MOS", ops++);
	OPCODE_map_insert(opmap, "ANS", ops++);
	OPCODE_map_insert(opmap, "ORS", ops++);
	OPCODE_map_insert(opmap, "SLS", ops++);
	OPCODE_map_insert(opmap, "SRS", ops++);
	OPCODE_map_insert(opmap, "XRS", ops++);
	OPCODE_map_insert(opmap, "ADI", ops++);
	OPCODE_map_insert(opmap, "SUI", ops++);
	OPCODE_map_insert(opmap, "MUI", ops++);
	OPCODE_map_insert(opmap, "DII", ops++);
	OPCODE_map_insert(opmap, "MOI", ops++);
	OPCODE_map_insert(opmap, "ANI", ops++);
	OPCODE_map_insert(opmap, "ORI", ops++);
	OPCODE_map_insert(opmap, "SLI", ops++);
	OPCODE_map_insert(opmap, "SRI", ops++);
	OPCODE_map_insert(opmap, "XRI", ops++);
	OPCODE_map_insert(opmap, "ADD", ops++);
	OPCODE_map_insert(opmap, "SUB", ops++);
	OPCODE_map_insert(opmap, "MUL", ops++);
	OPCODE_map_insert(opmap, "DIV", ops++);
	OPCODE_map_insert(opmap, "MOD", ops++);
	OPCODE_map_insert(opmap, "AND", ops++);
	OPCODE_map_insert(opmap, "OR", ops++);
	OPCODE_map_insert(opmap, "SHL", ops++);
	OPCODE_map_insert(opmap, "SHR", ops++);
	OPCODE_map_insert(opmap, "XOR", ops++);
	OPCODE_map_insert(opmap, "INV", ops++);
	OPCODE_map_insert(opmap, "COM", ops++);
	OPCODE_map_insert(opmap, "INI", ops++);
	OPCODE_map_insert(opmap, "COI", ops++);
	OPCODE_map_insert(opmap, "CMP", ops++);
	OPCODE_map_insert(opmap, "CMS", ops++);
	OPCODE_map_insert(opmap, "RET", ops++);
	OPCODE_map_insert(opmap, "REI", ops++);
	OPCODE_map_insert(opmap, "RES", ops++);
	OPCODE_map_insert(opmap, "ESC", ops++);
	OPCODE_map_insert(opmap, "CAL", ops++);
	OPCODE_map_insert(opmap, "PSH", ops++);
	OPCODE_map_insert(opmap, "PSS", ops++);
	OPCODE_map_insert(opmap, "POP", ops++);
	OPCODE_map_insert(opmap, "BNC", ops++);
	OPCODE_map_insert(opmap, "BNE", ops++);
	OPCODE_map_insert(opmap, "BEQ", ops++);
	OPCODE_map_insert(opmap, "BLT", ops++);
	OPCODE_map_insert(opmap, "BGT", ops++);
	OPCODE_map_insert(opmap, "BLE", ops++);
	OPCODE_map_insert(opmap, "BGE", ops++);
	OPCODE_map_insert(opmap, "JMP", ops++);
	OPCODE_map_insert(opmap, "JNE", ops++);
	OPCODE_map_insert(opmap, "JEQ", ops++);
	OPCODE_map_insert(opmap, "JLT", ops++);
	OPCODE_map_insert(opmap, "JGT", ops++);
	OPCODE_map_insert(opmap, "JLE", ops++);
	OPCODE_map_insert(opmap, "JGE", ops++);
	OPCODE_map_insert(opmap, "INT", ops++);
	OPCODE_map_insert(opmap, "INR", ops++);
}

void setup_register_map(REGISTER_map* regmap){
	REGISTER* regs = pool_request(regmap->mem, sizeof(REGISTER)*REGISTER_COUNT);
	for (REGISTER i = 0;i<REGISTER_COUNT;++i){
		regs[i] = i;
	}
	REGISTER_map_insert(regmap, "IP", regs++);
	REGISTER_map_insert(regmap, "SP", regs++);
	REGISTER_map_insert(regmap, "FP", regs++);
	REGISTER_map_insert(regmap, "SR", regs++);
	REGISTER_map_insert(regmap, "LR", regs++);
	REGISTER_map_insert(regmap, "CR", regs++);
	REGISTER_map_insert(regmap, "rA", regs++);
	REGISTER_map_insert(regmap, "rB", regs++);
	REGISTER_map_insert(regmap, "rC", regs++);
	REGISTER_map_insert(regmap, "rD", regs++);
	REGISTER_map_insert(regmap, "rE", regs++);
	REGISTER_map_insert(regmap, "rF", regs++);
	REGISTER_map_insert(regmap, "rG", regs++);
	REGISTER_map_insert(regmap, "rH", regs++);
	REGISTER_map_insert(regmap, "rI", regs++);
	REGISTER_map_insert(regmap, "rJ", regs++);
}

void setup_partition_map(REG_PARTITION_map* partmap){
	REG_PARTITION* parts = pool_request(partmap->mem, sizeof(REG_PARTITION)*PARTITION_COUNT);
	for (REG_PARTITION i = 0;i<PARTITION_COUNT;++i){
		parts[i] = i;
	}
	REG_PARTITION_map_insert(partmap, "Z]", parts++);
	REG_PARTITION_map_insert(partmap, "Y]", parts++);
	REG_PARTITION_map_insert(partmap, "X]", parts++);
	REG_PARTITION_map_insert(partmap, "W]", parts++);
	REG_PARTITION_map_insert(partmap, "Q]", parts++);
	REG_PARTITION_map_insert(partmap, "D]", parts++);
	REG_PARTITION_map_insert(partmap, "LO]", parts++);
	REG_PARTITION_map_insert(partmap, "HI]", parts++);
}

byte whitespace(char c){
	return (c=='\n' || c==' ' || c=='\r' || c == '\t');
}

#define LEXERR " [Line %lu] \033[1mLexing Error\033[0m "
#define PARSERR " \033[1mParsing Error\033[0m "
#define PARSERRFIX " at \033[1;31m %s\033[0m\n"

byte lex_cstr(compiler* const comp){
	comp->tokens = pool_request(comp->mem, sizeof(token));
	comp->token_count = 0;
	word line = 1;
	while (comp->str.i < comp->str.size){
		token* t = &comp->tokens[comp->token_count];
		t->text = &comp->str.text[comp->str.i];
		t->size = 1;
		t->type = NONE_TOKEN;
		byte negative = 0;
		char c = comp->str.text[comp->str.i];
		comp->str.i += 1;
		switch (c){
		case '\n':
			line += 1;
		case ' ':
		case '\t':
		case '\r':
			continue;
		case ';':
			c = comp->str.text[comp->str.i];
			comp->str.i += 1;
			byte multi = 0;
			if (c == ';'){
				multi = 1;
			}
			else if (c == '\n'){
				line += 1;
				continue;
			}
			while (comp->str.i < comp->str.size){
				c = comp->str.text[comp->str.i];
				comp->str.i += 1;
				if (multi == 1){
					if (c == ';'){
						c = comp->str.text[comp->str.i];
						if (c==';'){
							comp->str.i += 1;
							break;
						}
					}
				}
				else if (c == '\n'){
					line += 1;
					break;
				}
			}
			continue;
		case OPEN_CALL_TOKEN:
		case CLOSE_CALL_TOKEN:
		case OPEN_PUSH_TOKEN:
		case CLOSE_PUSH_TOKEN:
		case SUBLABEL_TOKEN:
		case LABEL_TOKEN:
			t->type = c;
			pool_request(comp->mem, sizeof(token));
			comp->token_count += 1;
			continue;
		case '-':
			c = comp->str.text[comp->str.i];
			ASSERT_LOCAL(c=='0', LEXERR " Expected numeric after '-'\n", line);
			comp->str.i += 1;
			t->size += 1;
			negative = 1;
		case '0':
			c = comp->str.text[comp->str.i];
			ASSERT_LOCAL(c == 'x' || c == 'X', LEXERR " Malformed numeric prefix\n", line);
			comp->str.i += 1;
			t->size += 1;
			int64_t number = 0;
			byte index = 0;
			while (comp->str.i < comp->str.size && index < 4){
				c = comp->str.text[comp->str.i];
				byte converted = 0;
				if (c >= '0' && c <= '9'){
					converted = (c - '0');
				}
				else if (c >= 'a' && c <= 'f'){
					converted = (c - 'a')+10;
				}
				else if (c >= 'A' && c <= 'F'){
					converted = (c - 'A')+10;
				}
				else{
					break;
				}
				number = (number << 4) | (converted & 0xF);
				comp->str.i += 1;
				index += 1;
				t->size += 1;
			}
			if (index <= sizeof(byte)*2){
				t->type = BYTE_HEX_NUMERIC_TOKEN;
				if (negative == 1){
				   	if (number > 127){
						t->type = SHORT_HEX_NUMERIC_TOKEN;
					}
					number *= -1;
				}
			}
			else if (index <= sizeof(uint16_t)*2){
				t->type = SHORT_HEX_NUMERIC_TOKEN;
				if (negative == 1){
					ASSERT_LOCAL(number <= 32767, LEXERR " Too many bytes provided for negative short numeric\n", line);
					number *= -1;
				}
			}
			else{
				ASSERT_LOCAL(0, LEXERR " Too many bytes provided for short hex numeric\n", line);
			}
			t->data.number = number;
			pool_request(comp->mem, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		ASSERT_LOCAL(c == '_' || isalpha(c), LEXERR " Expected identifier\n", line);
		t->type = IDENTIFIER_TOKEN;
		while (comp->str.i < comp->str.size){
			c = comp->str.text[comp->str.i];
			if (!isalnum(c) && c != '_'){
				break;
			}
			t->size += 1;
			comp->str.i += 1;
		}
		char copy = comp->str.text[comp->str.i];
		comp->str.text[comp->str.i] = '\0';
		REGISTER* r = REGISTER_map_access(&comp->regmap, (const char* const)t->text);
		if (r != NULL){
			t->type = REGISTER_TOKEN;
			t->data.reg = *r;
			comp->str.text[comp->str.i] = copy;
			pool_request(comp->mem, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		OPCODE* o = OPCODE_map_access(&comp->opmap, (const char* const)t->text);
		if (o != NULL){
			t->type = OPCODE_TOKEN;
			t->data.opcode = *o;
			comp->str.text[comp->str.i] = copy;
			pool_request(comp->mem, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		REG_PARTITION* p = REG_PARTITION_map_access(&comp->partmap, (const char* const)t->text);
		if (p != NULL){
			t->type = PART_TOKEN;
			t->data.part = *p;
			comp->str.text[comp->str.i] = copy;
			pool_request(comp->mem, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		comp->str.text[comp->str.i] = copy;
		pool_request(comp->mem, sizeof(token));
		comp->token_count += 1;
	}
	return 0;
}

word parse_register(compiler* const comp, word token_index, byte* r){
	token t = comp->tokens[token_index];
	ASSERT_LOCAL(t.type == REGISTER_TOKEN, PARSERR " Parsing register expected main register token" PARSERRFIX, t.text);
	byte base = t.data.reg;
	token_index += 1;
	t = comp->tokens[token_index];
	if (t.type != PART_TOKEN){
		*r = REG(FULL, base);
		return token_index;
	}
	token_index += 1;
	*r = REG(t.data.part, base);
	return token_index;
}

word parse_call_block(compiler* const comp, bsms* const sublabels, word token_index, call_tree* data){
	call_tree* last = data;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		token_index += 1;
		switch (t.type){
		case SHORT_HEX_NUMERIC_TOKEN:
		case BYTE_HEX_NUMERIC_TOKEN:
			data->type = NUMERIC_ARG;
			data->data.number = t.data.number;
			data->next = pool_request(comp->mem, sizeof(call_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case OPEN_PUSH_TOKEN:
			data->type = PUSH_ARG;
			data->data.push = pool_request(comp->mem, sizeof(data_tree));
			data->data.push->next = NULL;
			token_index = parse_push_block(comp, sublabels, token_index, data->data.push);
			ASSERT_ERR(0);
			data->next = pool_request(comp->mem, sizeof(call_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case OPEN_CALL_TOKEN:
			data->type = CALL_ARG;
			data->data.call = pool_request(comp->mem, sizeof(call_tree));
			data->data.call->next = NULL;
			token_index = parse_call_block(comp, sublabels, token_index, data->data.call);
			ASSERT_ERR(0);
			data->next = pool_request(comp->mem, sizeof(call_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case CLOSE_CALL_TOKEN:
			last->next = NULL;
			return token_index;
		case REGISTER_TOKEN:
			data->type = REG_ARG;
			data->data.reg = t.data.reg;
			token_index = parse_register(comp, token_index-1, &data->data.reg);
			data->next = pool_request(comp->mem, sizeof(call_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case SUBLABEL_TOKEN:
			t = comp->tokens[token_index];
			ASSERT_LOCAL(t.type == IDENTIFIER_TOKEN, PARSERR " Expected sublabel identifier following '.' token" PARSERRFIX, t.text);
			token_index += 1;
			data->type = SUBLABEL_ARG;
			data->data.labeling.label = t;
			code_tree* subloc = block_scope_check_member(&sublabels->map[sublabels->size-1], t, &data->data.labeling.dest_block);
			if (subloc != NULL){
				data->data.labeling.dest_block = subloc;
			}
			data->next = pool_request(comp->mem, sizeof(call_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case IDENTIFIER_TOKEN:
			data->type = LABEL_ARG;
			data->data.labeling.label = t;
			for (byte map_i = 0;map_i < comp->labels.size;++map_i){
				code_tree* loc = block_scope_check_member(&comp->labels.map[map_i], t, &data->data.labeling.dest_block);
				if (loc != NULL){
					data->data.labeling.dest_block = loc;
					break;
				}
			}
			data->next = pool_request(comp->mem, sizeof(call_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		default:
			ASSERT_LOCAL(0, PARSERR " Unexpected call argument token" PARSERRFIX, t.text);
		}
	}
	ASSERT_LOCAL(0, PARSERR " Unexpected end to procedure call\n");
	return 0;
}

word parse_byte_sequence(compiler* const comp, word token_index, data_tree* data){
	data->data.bytes.size = 0;
	word initial_index = token_index;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		if (t.type == SHORT_HEX_NUMERIC_TOKEN){
			data->data.bytes.size += 2;
		}
		else if (t.type == BYTE_HEX_NUMERIC_TOKEN){
			data->data.bytes.size += 1;
		}
		else{
			break;
		}
		token_index += 1;
	}
	data->data.bytes.raw = pool_request(comp->mem, data->data.bytes.size);
	word index = 0;
	for (word i = initial_index;i<token_index;++i){
		token t = comp->tokens[i];
		switch (t.type){
		case SHORT_HEX_NUMERIC_TOKEN:
			data->data.bytes.raw[index] = (t.data.number & 0xFF00) >> 8;
			index += 1;
			data->data.bytes.raw[index] = (t.data.number & 0xFF);
			index += 1;
			continue;
		case BYTE_HEX_NUMERIC_TOKEN:
			data->data.bytes.raw[index] = (t.data.number & 0xFF);
			index += 1;
			continue;
		default:
			ASSERT_LOCAL(0, PARSERR " Unexpected byte in sequence" PARSERRFIX, t.text);
		}
	}
	return token_index;
}

word parse_push_block(compiler* const comp, bsms* const sublabels, word token_index, data_tree* data){
	data_tree* last = data;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		token_index += 1;
		switch (t.type){
		case SHORT_HEX_NUMERIC_TOKEN:
		case BYTE_HEX_NUMERIC_TOKEN:
			data->type = BYTE_DATA;
			token_index = parse_byte_sequence(comp, token_index-1, data);
			data->next = pool_request(comp->mem, sizeof(data_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case OPEN_PUSH_TOKEN:
			data->type = NEST_DATA;
			data->data.nest = pool_request(comp->mem, sizeof(data_tree));
			token_index = parse_push_block(comp, sublabels, token_index, data->data.nest);
			ASSERT_ERR(0);
			data->next = pool_request(comp->mem, sizeof(data_tree));
			last = data;
			data = data->next;
			data->next = NULL;
			continue;
		case CLOSE_PUSH_TOKEN:
			last->next = NULL;
			return token_index;
		default:
			bsms_push(sublabels);
			bsms_push(&comp->labels);
			data->type = CODE_DATA;
			data->data.code = pool_request(comp->mem, sizeof(code_tree));
			data->data.code->labeling = NOT_LABELED;
			data->data.code->next = NULL;
			token_index = parse_code(comp, sublabels, token_index-1, data->data.code, CLOSE_PUSH_TOKEN);
			data->next = NULL;
			bsms_pop(&comp->labels);
			bsms_pop(sublabels);
			return token_index;
		}
	}
	ASSERT_LOCAL(0, PARSERR " Expected push block terminating character '}'\n");
	return 0;
}

word parse_3reg(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
	byte a = 0;
	byte b = 0;
	byte c = 0;
	token_index = parse_register(comp, token_index, &a);
	ASSERT_ERR(0);
	token_index = parse_register(comp, token_index, &b);
	ASSERT_ERR(0);
	token_index = parse_register(comp, token_index, &c);
	ASSERT_ERR(0);
	code->code.instructions[instruction_index] = op;
	code->code.instructions[instruction_index+1] = a;
	code->code.instructions[instruction_index+2] = b;
	code->code.instructions[instruction_index+3] = c;
	return token_index;
}

word parse_2reg(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
	byte a = 0;
	byte b = 0;
	token_index = parse_register(comp, token_index, &a);
	ASSERT_ERR(0);
	token_index = parse_register(comp, token_index, &b);
	ASSERT_ERR(0);
	code->code.instructions[instruction_index] = op;
	code->code.instructions[instruction_index+1] = a;
	code->code.instructions[instruction_index+2] = b;
	code->code.instructions[instruction_index+3] = 0;
	return token_index;
}

word parse_2reg_byte(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
	byte a = 0;
	byte b = 0;
	token_index = parse_register(comp, token_index, &a);
	ASSERT_ERR(0);
	token_index = parse_register(comp, token_index, &b);
	ASSERT_ERR(0);
	token t = comp->tokens[token_index];
	ASSERT_LOCAL(t.type == BYTE_HEX_NUMERIC_TOKEN, PARSERR " Expectd byte literal" PARSERRFIX, t.text);
	token_index += 1;
	code->code.instructions[instruction_index] = op;
	code->code.instructions[instruction_index+1] = a;
	code->code.instructions[instruction_index+2] = b;
	code->code.instructions[instruction_index+3] = (t.data.number & 0xFF);
	return token_index;
}

word parse_reg_short(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
	byte a = 0;
	token_index = parse_register(comp, token_index, &a);
	ASSERT_ERR(0);
	token t = comp->tokens[token_index];
	ASSERT_LOCAL(t.type == SHORT_HEX_NUMERIC_TOKEN || t.type == BYTE_HEX_NUMERIC_TOKEN, PARSERR " Expected numeric literal" PARSERRFIX, t.text);
	token_index += 1;
	code->code.instructions[instruction_index] = op;
	code->code.instructions[instruction_index+1] = a;
	code->code.instructions[instruction_index+2] = (t.data.number & 0xFF00) >> 8;
	code->code.instructions[instruction_index+3] = (t.data.number & 0xFF);
	return token_index;
}

word parse_instruction_block(compiler* const comp, bsms* const sublabels, word token_index, code_tree* code){
	token t = comp->tokens[token_index];
	ASSERT_LOCAL(t.type == OPCODE_TOKEN, PARSERR " Expected opcode in parse instruction start" PARSERRFIX, t.text);
	code->type = INSTRUCTION_BLOCK;
	code->code.instructions = pool_request(comp->mem, 4*BLOCK_START_SIZE);
	code->code.instruction_count = 0;
	word instruction_capacity = 4*BLOCK_START_SIZE;
	word instruction_index = 0;
	while (token_index < comp->token_count){
		t = comp->tokens[token_index];
		if (t.type != OPCODE_TOKEN){
			return token_index;
		}
		token_index += 1;
		OPCODE opc = t.data.opcode;
		if (opc >= BNC && opc <= JGE){
			t = comp->tokens[token_index];
			token_index += 1;
			byte triggered = INSTRUCTION_BLOCK;
			if (t.type == SUBLABEL_TOKEN){
				t = comp->tokens[token_index];
				ASSERT_LOCAL(t.type == IDENTIFIER_TOKEN, PARSERR " Expected identifier after sublabel token '.' in jump" PARSERRFIX, t.text);
				token_index += 1;
				triggered = INSTRUCTION_SUBJUMP;
				code_tree* loc = block_scope_check_member(&sublabels->map[sublabels->size-1], t, &code->dest_block);
				if (loc != NULL){
					code->dest_block = loc;
				}
			}
			else if (t.type == IDENTIFIER_TOKEN){
				triggered = INSTRUCTION_JUMP;
				for (byte map_i = 0;map_i<comp->labels.size;++map_i){
					code_tree* loc = block_scope_check_member(&comp->labels.map[map_i], t, &code->dest_block);
					if (loc != NULL){
						code->dest_block = loc;
						break;
					}
				}
			}
			if (triggered != INSTRUCTION_BLOCK){
				if (code->code.instruction_count > 0){
					code->next = pool_request(comp->mem, sizeof(code_tree));
					code = code->next;
					code->next = NULL;
					code->labeling = NOT_LABELED;
				}
				code->type = triggered;
				code->code.instruction_count = 1;
				code->code.instructions = pool_request(comp->mem, 4);
				code->code.instructions[0] = opc;
				code->code.instructions[1] = 0;//TODO mode?
				code->code.instructions[2] = 0;
				code->code.instructions[3] = 0;
				code->dest = t;
				return token_index;
			}
			switch (t.type){
			case REGISTER_TOKEN:
				byte link_reg = 0;
				token_index = parse_register(comp, token_index-1, &link_reg);
				ASSERT_ERR(0);
				code->code.instructions[instruction_index] = opc;
				code->code.instructions[instruction_index+1] = 0;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = link_reg;
				break;
			case SHORT_HEX_NUMERIC_TOKEN:
			case BYTE_HEX_NUMERIC_TOKEN:
				code->code.instructions[instruction_index] = opc;
				code->code.instructions[instruction_index+1] = 1;
				code->code.instructions[instruction_index+2] = (t.data.number & 0xFF00) >> 8;
				code->code.instructions[instruction_index+3] = (t.data.number & 0xFF);
				break;
			default:
				code->code.instructions[instruction_index] = opc;
				code->code.instructions[instruction_index+1] = 1;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = 0;
				token_index -= 1;
			}
		}
		else{
			switch (t.data.opcode){
			case LDA: case STA: case ADD: case SUB:
			case MUL: case DIV: case MOD: case AND:
			case OR: case SHL: case SHR: case XOR:
				token_index = parse_3reg(comp, t.data.opcode, instruction_index, token_index, code);
				break;
			case LDS: case STS: case ADS: case SUS:
			case MUS: case DIS: case MOS: case ANS:
			case ORS: case SLS: case SRS: case XRS:
			case CMS:
				token_index = parse_reg_short(comp, t.data.opcode, instruction_index, token_index, code);
				break;
			case MOV: case SWP: case ADI: case SUI:
			case MUI: case DII: case MOI: case ANI:
			case ORI: case SLI: case SRI: case XRI:
			case INV: case COM: case CMP:
				token_index = parse_2reg(comp, t.data.opcode, instruction_index, token_index, code);
				break;
			case LDI: case STB:
				token_index = parse_2reg_byte(comp, t.data.opcode, instruction_index, token_index, code);
				break;
			case INI: case COI: case RET: case PSH:
			case POP: case INR:
				byte r = 0;
				token_index = parse_register(comp, token_index, &r);
				ASSERT_ERR(0);
				code->code.instructions[instruction_index] = t.data.opcode;
				code->code.instructions[instruction_index+1] = r;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = 0;
				break;
			case RES: case PSS:
				token s = comp->tokens[token_index];
				ASSERT_LOCAL(s.type == SHORT_HEX_NUMERIC_TOKEN || s.type == BYTE_HEX_NUMERIC_TOKEN, PARSERR " Expected numeric literal" PARSERRFIX, s.text);
				token_index += 1;
				code->code.instructions[instruction_index] = t.data.opcode;
				code->code.instructions[instruction_index+1] = (s.data.number & 0xFF00) >> 8;
				code->code.instructions[instruction_index+2] = (s.data.number & 0xFF);
				code->code.instructions[instruction_index+3] = 0;
				break;
			case INT:
				token b = comp->tokens[token_index];
				ASSERT_LOCAL(b.type == BYTE_HEX_NUMERIC_TOKEN, PARSERR " Expected byte literal" PARSERRFIX, b.text);
				token_index += 1;
				code->code.instructions[instruction_index] = t.data.opcode;
				code->code.instructions[instruction_index+1] = (b.data.number & 0xFF);
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = 0;
				break;
			case NOP: case REI: case ESC: case CAL:
				code->code.instructions[instruction_index] = t.data.opcode;
				code->code.instructions[instruction_index+1] = 0;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = 0;
				break;
			default:
				ASSERT_LOCAL(0, PARSERR " Unknown opcode" PARSERRFIX, t.text);
			}
		}
		instruction_index += 4;
		code->code.instruction_count += 1;
		if (instruction_index >= instruction_capacity){
			pool_request(comp->mem, 4*code->code.instruction_count);
			instruction_capacity *= 2;
		}
	}
	return token_index;
}

word parse_code(compiler* const comp, bsms* const sublabels, word token_index, code_tree* ir, TOKEN terminator){
	code_tree* last = ir;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		token_index += 1;
		if (t.type == terminator){
			last->next = NULL;
			return token_index;
		}
		switch (t.type){
		case OPCODE_TOKEN:
			token_index = parse_instruction_block(comp, sublabels, token_index-1, ir);
			ASSERT_ERR(0);
			while (ir->next != NULL){
				ir = ir->next;
			}
			ir->next = pool_request(comp->mem, sizeof(code_tree));
			last = ir;
			ir = ir->next;
			ir->labeling = NOT_LABELED;
			ir->next = NULL;
			continue;
		case OPEN_CALL_TOKEN:
			ir->type = CALL_BLOCK;
			ir->data.call = pool_request(comp->mem, sizeof(call_tree));
			ir->data.call->next = NULL;
			token_index = parse_call_block(comp, sublabels, token_index, ir->data.call);
			ASSERT_ERR(0);
			ir->next = pool_request(comp->mem, sizeof(code_tree));
			last = ir;
			ir = ir->next;
			ir->labeling = NOT_LABELED;
			ir->next = NULL;
			continue;
		case OPEN_PUSH_TOKEN:
			ir->type = PUSH_BLOCK;
			ir->data.push = pool_request(comp->mem, sizeof(data_tree));
			ir->data.push->next = NULL;
			token_index = parse_push_block(comp, sublabels, token_index, ir->data.push);
			ASSERT_ERR(0);
			ir->next = pool_request(comp->mem, sizeof(code_tree));
			last = ir;
			ir = ir->next;
			ir->labeling = NOT_LABELED;
			ir->next = NULL;
			continue;
		case SUBLABEL_TOKEN:
			ASSERT_LOCAL(ir->labeling == NOT_LABELED, PARSERR " Double labeled instruction" PARSERRFIX, t.text);
			t = comp->tokens[token_index];
			token_index += 1;
			ASSERT_LOCAL(t.type == IDENTIFIER_TOKEN, PARSERR " Expected label after '.'" PARSERRFIX, t.text);
			token sublabel_name = t;
			t = comp->tokens[token_index];
			token_index += 1;
			ASSERT_LOCAL(t.type == LABEL_TOKEN, PARSERR " Unexpected label" PARSERRFIX, t.text);
			ir->labeling = SUBLABELED;
			ir->label = sublabel_name;
			byte duplicate_sublabel = block_scope_add_member(&sublabels->map[sublabels->size-1], sublabel_name, ir);
			ASSERT_LOCAL(duplicate_sublabel == 0, PARSERR " Duplicate sublabel " PARSERRFIX, sublabel_name.text);
			continue;
		case IDENTIFIER_TOKEN:
			ASSERT_LOCAL(ir->labeling == NOT_LABELED, PARSERR " Double labeled instruction" PARSERRFIX, t.text);
			token label_name = t;
			t = comp->tokens[token_index];
			token_index += 1;
			ASSERT_LOCAL(t.type == LABEL_TOKEN, PARSERR " Unexpected label" PARSERRFIX, t.text);
			ir->labeling = LABELED;
			ir->label = label_name;
			block_scope_map* submap = &sublabels->map[sublabels->size-1];
			remaining_labels(comp, submap);
			ASSERT_ERR(0);
			if (sublabels->size == 1){
				pool_empty(submap->mem);
			}
			block_scope_map_empty(submap);
			byte duplicate_label = block_scope_add_member(&comp->labels.map[comp->labels.size-1], label_name, ir);
			ASSERT_LOCAL(duplicate_label == 0, PARSERR " Duplicate label " PARSERRFIX, label_name.text);
			continue;
		default:
			ASSERT_LOCAL(0, PARSERR " Unexpected token type" PARSERRFIX, t.text);
		}
	}
	last->next = NULL;
	return token_index;
}

byte block_scope_add_member(block_scope_map* const block, token t, code_tree* member){
	char* name = t.text;
	word size = t.size;
	char save = name[size];
	name[size] = '\0';
	block_scope* node = block_scope_map_access(block, name);
	name[size] = save;
	if (node != NULL){
		if (node->type == FULFILLED_MEMBER){
			return 1;
		}
		node->type = FULFILLED_MEMBER;
		while (node != NULL){
			*node->ref = member;
			node = node->next;
		}
		return 0;
	}
	node = pool_request(block->mem, sizeof(block_scope));
	node->type = FULFILLED_MEMBER;
	node->next = NULL;
	node->label = member;
	char* new_name = pool_request(block->mem, size);
	strncpy(new_name, name, size);
	block_scope_map_insert(block, new_name, node);
	return 0;
}

code_tree* block_scope_check_member(block_scope_map* const block, token t, code_tree** ref){
	char* name = t.text;
	word size = t.size;
	char save = name[size];
	name[size] = '\0';
	block_scope* node = block_scope_map_access(block, name);
	name[size] = save;
	if (node != NULL){
		if (node->type == FULFILLED_MEMBER){
			return node->label;
		}
		block_scope* temp = node->next;
		node->next = pool_request(block->mem, sizeof(block_scope));
		node = node->next;
		node->next = temp;
		node->type = PENDING_MEMBER;
		node->ref = ref;
		node->pending_source = t;
		node->label = NULL;
		return NULL;
	}
	node = pool_request(block->mem, sizeof(block_scope));
	node->type = PENDING_MEMBER;
	node->next = NULL;
	node->label = NULL;
	node->pending_source = t;
	node->ref = ref;
	char* new_name = pool_request(block->mem, size);
	strncpy(new_name, name, size);
	block_scope_map_insert(block, new_name, node);
	return NULL;
}

void bsms_push(bsms* stack){
	if (stack->size >= stack->capacity){
		byte old_cap = stack->capacity;
		stack->capacity *= 2;
		block_scope_map* old = stack->map;
		stack->map = pool_request(old->mem, sizeof(block_scope_map)*stack->capacity);
		for (byte i = 0;i<old_cap;++i){
			stack->map[i] = old[i];
		}
	}
	stack->map[stack->size] = block_scope_map_init(stack->map[0].mem);
	stack->size += 1;
}

void bsms_pop(bsms* stack){
	stack->size -= 1;
}

byte check_label_bucket(compiler* const comp, block_scope_map_bucket* bucket){
	if (bucket->tag == BUCKET_EMPTY){
		return 0;
	}
	ASSERT_LOCAL(bucket->value->type == FULFILLED_MEMBER, PARSERR " Unresolved label jump" PARSERRFIX, bucket->value->pending_source.text);
	if (bucket->left != NULL){
		check_label_bucket(comp, bucket->left);
	}
	if (bucket->right != NULL){
		check_label_bucket(comp, bucket->right);
	}
	return 0;
}

byte remaining_labels(compiler* const comp, block_scope_map* const block){
	HASHMAP_ITERATE(i){
		block_scope_map_bucket* bucket = &block->buckets[i];
		check_label_bucket(comp, bucket);
		ASSERT_ERR(0);
	}
	return 0;
}

byte parse_tokens(compiler* const comp){
	word token_index = 0;
	comp->ir = pool_request(comp->mem, sizeof(code_tree));
	comp->ir->labeling = NOT_LABELED;
	comp->ir->next = NULL;
	{
		comp->labels.map = pool_request(comp->mem, sizeof(block_scope_map)*PUSH_LABEL_SCOPE_LIMIT);
		comp->labels.map[0] = block_scope_map_init(comp->mem);
		comp->labels.size = 1;
		comp->labels.capacity = PUSH_LABEL_SCOPE_LIMIT;
	}
	pool sublabel_pool = pool_alloc(AUX_SIZE, POOL_STATIC);
	bsms sublabels = {
		.map = pool_request(comp->mem, sizeof(block_scope_map)*PUSH_LABEL_SCOPE_LIMIT),
		.size = 1,
		.capacity=PUSH_LABEL_SCOPE_LIMIT
	};
	sublabels.map[0] = block_scope_map_init(&sublabel_pool);
	parse_code(comp, &sublabels, token_index, comp->ir, NONE_TOKEN);
	pool_dealloc(&sublabel_pool);
	remaining_labels(comp, &comp->labels.map[comp->labels.size-1]);
	ASSERT_ERR(0);
	return 0;
}

#define INDENT_SHOW(d)\
	for (byte di = 0;di<d;++di){\
		printf(" ");\
	}

byte show_call(compiler* const comp, call_tree* call, word depth){
	ASSERT_LOCAL(call != NULL, " call started as null\n");
	byte first = 1;
	while (call != NULL){
		char save;
		printf("\n");
		if (first == 1){
			INDENT_SHOW(depth) printf("(PROCEDURE)");
			first = 0;
		}
		else{
			INDENT_SHOW(depth) printf("(ARG)");
		}
		switch (call->type){
		case CALL_ARG:
			INDENT_SHOW(depth) printf("CALL:\n");
			show_call(comp, call->data.call, depth+1);
			ASSERT_ERR(0);
			break;
		case PUSH_ARG:
			INDENT_SHOW(depth) printf("PUSH:\n");
			show_data(comp, call->data.push, depth+1);
			ASSERT_ERR(0);
			break;
		case REG_ARG:
			INDENT_SHOW(depth) printf("REGISTER: %u\n", call->data.reg);
			break;
		case LABEL_ARG:
			save = call->data.labeling.label.text[call->data.labeling.label.size];
			call->data.labeling.label.text[call->data.labeling.label.size] = '\0';
			INDENT_SHOW(depth) printf("LABEL: %s\n", call->data.labeling.label.text);
			call->data.labeling.label.text[call->data.labeling.label.size] = save;
			break;
		case SUBLABEL_ARG:
			save = call->data.labeling.label.text[call->data.labeling.label.size];
			call->data.labeling.label.text[call->data.labeling.label.size] = '\0';
			INDENT_SHOW(depth) printf("SUBLABEL: .%s\n", call->data.labeling.label.text);
			call->data.labeling.label.text[call->data.labeling.label.size] = save;
			break;
		case NUMERIC_ARG:
			INDENT_SHOW(depth) printf("NUMERIC: %lx\n", call->data.number);
			break;
		}
		call = call->next;
	}
	return 0;
}

byte show_data(compiler* const comp, data_tree* data, word depth){
	ASSERT_LOCAL(data != NULL, " push started as null\n");
	while (data != NULL){
		printf("\n");
		switch (data->type){
		case BYTE_DATA:
			INDENT_SHOW(depth) printf("BYTE SEQUENCE:\n");
			for (word i = 0;i<data->data.bytes.size;){
				word n = i+4;
				INDENT_SHOW(depth);
				for (;i<n;++i){
					printf("%02x ", data->data.bytes.raw[i]);
				}
				if (i%4 == 0){
					printf("\n");
				}
			}
			for (byte k = (1+data->data.bytes.size) % sizeof(word);k>0;--k){
				printf("00 ");
			}
			printf("\n");
			break;
		case NEST_DATA:
			INDENT_SHOW(depth) printf("PUSH:\n");
			show_data(comp, data->data.nest, depth+1);
			break;
		case CODE_DATA:
			INDENT_SHOW(depth) printf("CODE:\n");
			show_block(comp, data->data.code, depth+1);
			ASSERT_ERR(0);
			break;
		}
		data = data->next;
	}
	return 0;
}

byte show_block(compiler* const comp, code_tree* code, word depth){
	ASSERT_LOCAL(code != NULL, " block started as null\n");
	while (code != NULL){
		printf("\n");
		char save;
		if (code->labeling == LABELED){
			save = code->label.text[code->label.size];
			code->label.text[code->label.size] = '\0';
			INDENT_SHOW(depth) printf("LABELED %s:\n", code->label.text);
			code->label.text[code->label.size] = save;
		}
		else if (code->labeling == SUBLABELED){
			save = code->label.text[code->label.size];
			code->label.text[code->label.size] = '\0';
			INDENT_SHOW(depth) printf("SUBLABELED .%s:\n", code->label.text);
			code->label.text[code->label.size] = save;
		}
		switch (code->type){
		case INSTRUCTION_BLOCK:
			INDENT_SHOW(depth) printf("INSTRUCTION BLOCK:\n");
			for (word i = 0;i<code->code.instruction_count;++i){
				word inst = i*4;
				word n = inst+4;
				INDENT_SHOW(depth);
				for (;inst<n;++inst){
					printf("%02x ", code->code.instructions[inst]);
				}
				printf("\n");
			}
			break;
		case INSTRUCTION_JUMP:
			INDENT_SHOW(depth) printf("JUMP:\n");
			for (byte i = 0;i<4;++i){
				printf("%02x ", code->code.instructions[i]);
			}
			save = code->dest.text[code->dest.size];
			code->dest.text[code->dest.size] = '\0';
			printf("-> %s\n", code->dest.text);
			code->dest.text[code->dest.size] = save;
			break;
		case INSTRUCTION_SUBJUMP:
			INDENT_SHOW(depth) printf("SUB JUMP:\n");
			for (byte i = 0;i<4;++i){
				printf("%02x ", code->code.instructions[i]);
			}
			save = code->dest.text[code->dest.size];
			code->dest.text[code->dest.size] = '\0';
			printf("-> .%s\n", code->dest.text);
			code->dest.text[code->dest.size] = save;
			break;
		case CALL_BLOCK:
			INDENT_SHOW(depth) printf("CALL:\n");
			show_call(comp, code->data.call, depth+1);
			ASSERT_ERR(0);
			break;
		case PUSH_BLOCK:
			INDENT_SHOW(depth) printf("PUSH:\n");
			show_data(comp, code->data.push, depth+1);
			ASSERT_ERR(0);
			break;
		}
		code = code->next;
	}
	return 0;
}

void show_tokens(compiler* const comp){
	for (word i = 0;i<comp->token_count;++i){
		printf("[ ");
		switch (comp->tokens[i].type){
		case OPCODE_TOKEN:
			printf("OPCODE %u ", comp->tokens[i].data.opcode);
			break;
		case REGISTER_TOKEN:
			printf("REGISTER %u ", comp->tokens[i].data.reg);
			break;
		case PART_TOKEN:
			printf("PARTITION %u ", comp->tokens[i].data.part);
			break;
		case OPEN_CALL_TOKEN:
			printf("OPEN_CALL");
			break;
		case CLOSE_CALL_TOKEN:
			printf("CLOSE_CALL ");
			break;
		case OPEN_PUSH_TOKEN:
			printf("OPEN_PUSH ");
			break;
		case CLOSE_PUSH_TOKEN:
			printf("CLOSE_PUSH ");
			break;
		case SUBLABEL_TOKEN:
			printf("SUBLABEL ");
			break;
		case LABEL_TOKEN:
			printf("LABEL ");
			break;
		case SHORT_HEX_NUMERIC_TOKEN:
			printf("SHORT ");
			break;
		case BYTE_HEX_NUMERIC_TOKEN:
			printf("BYTE ");
			break;
		case IDENTIFIER_TOKEN:
			printf("IDENTIFIER ");
			break;
		case NONE_TOKEN:
			printf("????? ");
			break;
		}
		char save = comp->tokens[i].text[comp->tokens[i].size];
		comp->tokens[i].text[comp->tokens[i].size] = '\0';
		printf("'%s' ] ", comp->tokens[i].text);
		comp->tokens[i].text[comp->tokens[i].size] = save;
		if ((i+1) % 8 == 0){
			printf("\n");
		}
	}
	printf("\n");
}

byte compile_cstr(compiler* const comp){
	lex_cstr(comp);
	ASSERT_ERR(0);
	show_tokens(comp);
	parse_tokens(comp);
	ASSERT_ERR(0);
	show_block(comp, comp->ir, 0);
	return 0;
}

void compile_file(char* infile, char* outfile){
	FILE* fd = fopen(infile, "r");
	if (fd == NULL){
		fprintf(stderr, "File '%s' not found\n", infile);
		return;
	}
	pool mem = pool_alloc(ARENA_SIZE, POOL_STATIC);
	if (mem.buffer == NULL){
		fprintf(stderr, "Couldnt allocate arena for compilation\n");
		fclose(fd);
		return;
	}
	word read_bytes = fread(mem.buffer, sizeof(byte), READ_BUFFER_SIZE, fd);
	fclose(fd);
	if (read_bytes == READ_BUFFER_SIZE){
		fprintf(stderr, "File too big\n");
		pool_dealloc(&mem);
		return;
	}
	string str = {.size=read_bytes, .i=0};
	str.text = pool_request(&mem, read_bytes);
	if (str.text == NULL){
		fprintf(stderr, "Unable to allocate read buffer\n");
		return;
	}
	string buf = {.size=WRITE_BUFFER_SIZE, .i=0};
	buf.text = pool_request(&mem, WRITE_BUFFER_SIZE);
	if (buf.text == NULL){
		fprintf(stderr, "Unable to allocate write buffer\n");
		pool_dealloc(&mem);
		return;
	}
	OPCODE_map opmap = OPCODE_map_init(&mem);
	REGISTER_map regmap = REGISTER_map_init(&mem);
	REG_PARTITION_map partmap = REG_PARTITION_map_init(&mem);
	setup_opcode_map(&opmap);
	setup_register_map(&regmap);
	setup_partition_map(&partmap);
	compiler comp = {
		.str = str,
		.buf = buf,
		.opmap = opmap,
		.regmap = regmap,
		.partmap = partmap,
		.mem = &mem,
		.err = pool_request(&mem, ERROR_BUFFER)
	};
	*comp.err = 0;
	compile_cstr(&comp);
	if (*comp.err != 0){
		fprintf(stderr, "Unable to compile '%s'\n", infile);
		fprintf(stderr, comp.err);
		fprintf(stderr, "\033[0m\n");
		pool_dealloc(&mem);
		return;
	}
	fd = fopen(outfile, "w");
	if (fd == NULL){
		fprintf(stderr, "Unable to open file '%s' for writing\n", outfile);
		pool_dealloc(&mem);
		return;
	}
	fwrite(comp.buf.text, 1, comp.buf.i, fd);
	fclose(fd);
	pool_dealloc(&mem);
}

void setup_registers(){
	for (uint8_t r = 0;r<REGISTER_COUNT;++r){
		for (uint8_t i = 0;i<4;++i){
			quar[(r*4)+i] = (uint16_t*)(&reg[r])+(3-i);
		}
		half[r] = (uint32_t*)(&reg[r]);
		lo[r] = (byte*)(&reg[r]);
		hi[r] = (byte*)(&reg[r])+1;
	}
	reg[SP] = MEMORY_SIZE-1;
	reg[FP] = MEMORY_SIZE-1;
	reg[CR] = MEMORY_SIZE-1;
	reg[IP] = 0;
}

void flash_rom(byte* buffer, uint64_t size){
	for (uint64_t i = 0;i<size;++i){
		mem[PROGRAM_START+i] = buffer[i];
	}
}

void demo(){
	byte cc[] = {
		CAL_,
			LDS_(REG(L16, R3), 0xDEAD),
			LDS_(REG(LM16, R3), 0xBEEF),
			LDS_(REG(RM16, R3), 0xFACE),
			LDS_(REG(R16, R3), 0xCAFE),
		PSH_(REG(FULL, R3)),
			LDS_(REG(L16, R3), 0xC0FF),
			LDS_(REG(LM16, R3), 0xEEFA),
			LDS_(REG(RM16, R3), 0xCEDD),
			LDS_(REG(R16, R3), 0xEAD5),
		PSH_(REG(FULL, R3)),
		BNC_(1, 0x3),
		JMP_(1, 0x9),
		NOP_,
			MOV_(REG(FULL, R0), REG(FULL, FP)),
			ADS_(REG(FULL, R0), 0x11),
			LDI_(REG(HALF, R2), REG(FULL, R0), 0x0),
			ADS_(REG(FULL, R0), 0x4),
			CMP_(REG(FULL, R0), REG(FULL, CR)),
			JLT_(1, -0x3),
			RET_(REG(FULL, R2)),
		POP_(REG(FULL, R4)),
		LDS_(REG(FULL, R2), 0xDEAD),
		NOP_, NOP_, NOP_, NOP_
	};
	setup_registers();
	flash_rom(cc, 128);
	interpret();
	return;
}

void show_binary(char* filename){
	FILE* fd = fopen(filename, "rb");
	if (fd == NULL){
		fprintf(stderr, "Unable to open file '%s'\n", filename);
		return;
	}
	byte* buffer = malloc(WRITE_BUFFER_SIZE);
	word size = fread(buffer, sizeof(byte), WRITE_BUFFER_SIZE, fd);
	fclose(fd);
	if (size >= WRITE_BUFFER_SIZE){
		fprintf(stderr, "Rom image too long\n");
		free(buffer);
		return;
	}
	for (word i = 0;i<size;++i){
		printf("%02x ", buffer[i]);
		if ((i+1)%4==0){
			printf("\n");
		}
	}
	printf("\n");
	free(buffer);
}

void run_rom(char* filename){
	FILE* fd = fopen(filename, "rb");
	if (fd == NULL){
		fprintf(stderr, "Unable to open file '%s'\n", filename);
		return;
	}
	byte* buffer = malloc(WRITE_BUFFER_SIZE);
	word size = fread(buffer, sizeof(byte), WRITE_BUFFER_SIZE, fd);
	fclose(fd);
	if (size >= WRITE_BUFFER_SIZE){
		fprintf(stderr, "Rom image too long\n");
		free(buffer);
		return;
	}
	setup_registers();
	flash_rom(buffer, size);
	interpret();
	free(buffer);
}

int32_t main(int argc, char** argv){
	compile_file("full_syntax.src", "full_syntax.rom");
	return 0;
	if (argc <= 1){
		printf(" -h for help\n");
		return 0;
	}
	if (strncmp(argv[1], "-h", TOKEN_MAX) == 0){
		printf(" compile program : -c infile.src -o outfile.rom\n");
		printf("     run program : -r file.rom\n");
		printf("     show binary : -s file.rom\n");
		printf("    demo program : -d\n");
		return 0;
	}
	if (strncmp(argv[1], "-c", TOKEN_MAX) == 0){
		if (argc != 5){
			printf(" Wrong number of arguments for compilation: -c infile.src -o outfile.rom\n");
			return 0;
		}
		if (strncmp(argv[3], "-o", TOKEN_MAX) != 0){
			printf(" Compilation requires '-o' to designate output file name\n");
			return 0;
		}
		compile_file(argv[2], argv[4]);
		return 0;
	}
	if (strncmp(argv[1], "-r", TOKEN_MAX) == 0){
		if (argc != 3){
			printf(" Needs a rom image name\n");
			return 0;
		}
		run_rom(argv[2]);
		return 0;
	}
	if (strncmp(argv[1], "-s", TOKEN_MAX) == 0){
		if (argc != 3){
			printf(" Needs a rom image name\n");
			return 0;
		}
		show_binary(argv[2]);
		return 0;
	}
	if (strncmp(argv[1], "-d", TOKEN_MAX) == 0){
		demo();
		return 0;
	}
	printf(" Unknown arguments, entry -h for help\n");
	return 0;
}

