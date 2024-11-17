#include <stdio.h>
#include <stdlib.h>
#include "machine.h"
#include "hashmap.h"

#pragma GCC diagnostic ignored "-Wsequence-point"

MAP_IMPL(OPCODE)
MAP_IMPL(REGISTER)
MAP_IMPL(REG_PARTITION)

#define SHORT(lit) (lit&0xFF00)>>8, lit&0xFF
#define NOP_                   NOP, 0,    0, 0
#define LDS_(dest, lit)        LDS, dest, SHORT(lit)
#define LDB_(dest, lit)        LDB, dest, lit, 0
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
#define REB_(tar)              REB, tar,  0, 0
#define ESC_                   ESC, 0,    0, 0
#define PSH_(tar)              PSH, tar,  0, 0
#define PSS_(tar)              PSS, SHORT(tar), 0
#define PSB_(tar)              PSB, tar,  0, 0
#define POP_(dst)              POP, dst,  0, 0
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
	SHOW_REG(RA);
	printf("\n");
}

void show_mem(){
	printf("\033[4mProgram / Stack\033[0m\n");
	printf("\033[1;32m");
	for (byte i = 0;i<INSTRUCTION_WIDTH*8;){
		printf("%016lx | ", reg[IP]+i);
		byte m = i+4;
		for (;i<m;++i){
			printf("%02x \033[0m\033[1m", mem[reg[IP]+i]);
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
		word m = i+4;
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
		reg[SP] -= 3;\
		*(uint32_t*)(&mem[reg[SP]]) = *half[r];\
		reg[SP] -= 1;\
		break;\
	case LO:\
		*(byte*)(&mem[reg[SP]]) = *lo[r];\
		reg[SP] -= 1;\
		break;\
	case HI:\
		*(byte*)(&mem[reg[SP]]) = *hi[r];\
		reg[SP] -= 1;\
		break;\
	default:\
		reg[SP] -= 1;\
		*(uint16_t*)(&mem[reg[SP]]) = *quar[(r*4)+partition];\
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
		*half[r] = *(uint32_t*)(&mem[reg[SP]]);\
		reg[SP] += 3;\
		break;\
	case LO:\
		reg[SP] += 1;\
		*lo[r] = *(byte*)(&mem[reg[SP]]);\
		break;\
	case HI:\
		reg[SP] += 1;\
		*hi[r] = *(byte*)(&mem[reg[SP]]);\
		break;\
	default:\
		reg[SP] += 1;\
		*quar[(r*4)+partition] = *(uint16_t*)(&mem[reg[SP]]);\
		reg[SP] += 1;\
		break;\
	}\
}

#define ALU_S(operator)\
	byte dst = NEXT;\
	uint16_t right = SHORT_LITERAL;\
	word left; ACCESS_REG(left, dst);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += INSTRUCTION_WIDTH;

#define ALU_I(operator)\
	byte dst = NEXT;\
	byte src = NEXT;\
	word left; ACCESS_REG(left, dst);\
	word right; ACCESS_REG(right, src);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += INSTRUCTION_WIDTH;

#define ALU(operator)\
	byte dst = NEXT;\
	byte src_a = NEXT;\
	byte src_b = NEXT;\
	word left; ACCESS_REG(left, src_a);\
	word right; ACCESS_REG(right, src_b);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += INSTRUCTION_WIDTH;

#define ALU_UI(operator)\
	byte tar = NEXT;\
	word val; ACCESS_REG(val, tar);\
	LOAD_REG(tar, operator val);\
	reg[IP] += INSTRUCTION_WIDTH;

#define ALU_U(operator)\
	byte tar = NEXT;\
	byte src = NEXT;\
	word val; ACCESS_REG(val, src);\
	LOAD_REG(tar, operator val);\
	reg[IP] += INSTRUCTION_WIDTH;

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
	ACCESS_REG(reg[IP], adr);

#define JUMP_REG\
	ip += 1;\
	byte adr = NEXT;\
	ACCESS_REG(reg[IP], adr);

#define JUMP\
	int16_t offset = SHORT_LITERAL;\
	reg[IP] += offset;

void interpret(){
	while (1){
		show_machine();
		getc(stdin);
		word ip = reg[IP];
		byte op = mem[ip];
		switch (op){
		case NOP: { if (1) {return;} else {reg[IP] += INSTRUCTION_WIDTH;}} break;
		case LDS: { byte b = NEXT; LOAD_REG(b, SHORT_LITERAL); reg[IP] += INSTRUCTION_WIDTH; } break;
		case LDB: { byte b = NEXT; LOAD_REG(b, NEXT); reg[IP] += INSTRUCTION_WIDTH; } break;
		case LDA: {
				byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				word o; ACCESS_REG(o, off);
				LOAD_REG_ADDR(dst, a+o);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case LDI: {
				byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				LOAD_REG_ADDR(dst, a+off);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case STS: { byte src = NEXT; uint16_t adr = SHORT_LITERAL; STORE_REG(adr, src); reg[IP] += INSTRUCTION_WIDTH; } break;
		case STA: {
				byte src = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				word o; ACCESS_REG(o, off);
				STORE_REG(a+o, src);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case STB: {
				byte src = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				STORE_REG(a+off, src);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case MOV: {
				byte dst = NEXT; byte src = NEXT;
				word s; ACCESS_REG(s, src);
				LOAD_REG(dst, s);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case SWP: {
				byte a = NEXT; byte b = NEXT;
				word s; ACCESS_REG(s, a);
				word d; ACCESS_REG(d, b);
				LOAD_REG(a, d); LOAD_REG(b, s);
				reg[IP] += INSTRUCTION_WIDTH;
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
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case CMS: {
				byte a = NEXT; uint16_t right = SHORT_LITERAL;
				word left; ACCESS_REG(left, a);
				COMPARE_FLAGS(left, right);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case RET: {
				byte tar = NEXT;
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP));
				POP_REG(REG(FULL,IP));
				PUSH_REG(tar);
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case REI: {
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP));
				POP_REG(REG(FULL,IP));
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case RES: {
				uint16_t tar = SHORT_LITERAL;
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP))
				POP_REG(REG(FULL,IP))
				reg[SP] -= 1;
				*(uint16_t*)(&mem[reg[SP]]) = tar;
				reg[SP] -= 1;
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case REB: {
				byte tar = NEXT;
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP))
				POP_REG(REG(FULL,IP))
				*(byte*)(&mem[reg[SP]]) = tar;
				reg[SP] -= 1;
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case ESC: {
				reg[SP] = reg[FP];
				POP_REG(REG(FULL,FP))
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case PSH: { byte tar = NEXT; PUSH_REG(tar); reg[IP] += INSTRUCTION_WIDTH; } break;
		case PSS: {
				uint16_t lit = SHORT_LITERAL;
				reg[SP] -= 1;
				*(uint16_t*)(&mem[reg[SP]]) = lit;
				reg[SP] -= 1;
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case PSB: {
				byte b = NEXT;
				*(byte*)(&mem[reg[SP]]) = b;
				reg[SP] -= 1;
				reg[IP] += INSTRUCTION_WIDTH;
			} break;
		case POP: { byte tar = NEXT; POP_REG(tar); reg[IP] += INSTRUCTION_WIDTH; } break;
		case BNC: {
				byte mode = NEXT;
				if (mode == 0) { BRANCH_LINK; } else { JUMP; }
			} break;
		case BNE: {
				if (((reg[SR] & ZERO) == 0) || ((reg[SR] & CARRY) != 0))
				{ byte mode = NEXT; if (mode == 0) { BRANCH_LINK; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case BEQ: {
				if (((reg[SR] & ZERO) != 0) && ((reg[SR] & CARRY) == 0))
				{ byte mode = NEXT; if (mode == 0) { BRANCH_LINK; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case BLT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) != 0))
				{ byte mode = NEXT; if (mode == 0) { BRANCH_LINK; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case BGT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) == 0))
				{ byte mode = NEXT; if (mode == 0) { BRANCH_LINK; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case BLE: {
				if ((reg[SR] & ZERO) != (reg[SR] & CARRY))
				{ byte mode = NEXT; if (mode == 0) { BRANCH_LINK; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case BGE: {
				if ((reg[SR] & CARRY) == 0)
				{ byte mode = NEXT; if (mode == 0) { BRANCH_LINK; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case JMP: {
				byte mode = NEXT;
				if (mode == 0) { JUMP_REG; } else { JUMP; }
			} break;
		case JNE: {
				if (((reg[SR] & ZERO) == 0) || ((reg[SR] & CARRY) != 0))
				{ byte mode = NEXT; if (mode == 0) { JUMP_REG; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case JEQ: {
				if (((reg[SR] & ZERO) != 0) && ((reg[SR] & CARRY) == 0))
				{ byte mode = NEXT; if (mode == 0) { JUMP_REG; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case JLT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) != 0))
				{ byte mode = NEXT; if (mode == 0) { JUMP_REG; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case JGT: {
				if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) == 0))
				{ byte mode = NEXT; if (mode == 0) { JUMP_REG; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case JLE: {
				if ((reg[SR] & ZERO) != (reg[SR] & CARRY))
				{ byte mode = NEXT; if (mode == 0) { JUMP_REG; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case JGE: {
				if ((reg[SR] & CARRY) == 0)
				{ byte mode = NEXT; if (mode == 0) { JUMP_REG; } else { JUMP; }}
				else { reg[IP] += INSTRUCTION_WIDTH; }
			} break;
		case INT: { reg[IP] += INSTRUCTION_WIDTH; } break;
		case INR: { reg[IP] += INSTRUCTION_WIDTH; } break;
		default:
			printf("Unknown upcode\n");
			return;
		}
	}
}

#define ASSERT_LOCAL(b, m)\
	if (!(b)){\
		snprintf(comp->err, ERROR_BUFFER, m);\
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
	OPCODE_map_insert(opmap, "LDB", ops++);
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
	OPCODE_map_insert(opmap, "REB", ops++);
	OPCODE_map_insert(opmap, "ESC", ops++);
	OPCODE_map_insert(opmap, "PSH", ops++);
	OPCODE_map_insert(opmap, "PSS", ops++);
	OPCODE_map_insert(opmap, "PSB", ops++);
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
	REGISTER_map_insert(regmap, "RA", regs++);
	REGISTER_map_insert(regmap, "RB", regs++);
	REGISTER_map_insert(regmap, "RC", regs++);
	REGISTER_map_insert(regmap, "RD", regs++);
	REGISTER_map_insert(regmap, "RE", regs++);
	REGISTER_map_insert(regmap, "RF", regs++);
	REGISTER_map_insert(regmap, "RG", regs++);
	REGISTER_map_insert(regmap, "RH", regs++);
	REGISTER_map_insert(regmap, "RI", regs++);
	REGISTER_map_insert(regmap, "RJ", regs++);
	REGISTER_map_insert(regmap, "RK", regs++);
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

byte parse_partition(compiler* const comp){
	while (comp->str.i < comp->str.size){
		char c = comp->str.text[comp->str.i];
		switch (c){
		case ' ':
		case '\n':
		case '\t':
		case '\r':
			comp->str.i += 1;
			continue;
		}
		uint32_t hash = 5381;
		int16_t s;
		word start = comp->str.i;
		while (comp->str.i < comp->str.size){
			c = comp->str.text[comp->str.i];
			if (whitespace(c)){
				break;
			}
			s = comp->str.text[comp->str.i];
			hash = ((hash<<5)+hash)+s;
			comp->str.i += 1;
		}
		char copy = comp->str.text[comp->str.i];
		comp->str.text[comp->str.i] = '\0';
		REG_PARTITION* r = REG_PARTITION_map_access_by_hash(&comp->partmap, hash, (const char* const)comp->str.text+start);
		comp->str.text[comp->str.i] = copy;
		comp->str.i += 1;
		if (r == NULL){
			return 1;
		}
		return *r;
	}
	ASSERT_LOCAL(0, "Unexpected EOF\n");
}

byte parse_register(compiler* const comp){
	while (comp->str.i < comp->str.size){
		char c = comp->str.text[comp->str.i];
		switch (c){
		case ' ':
		case '\n':
		case '\t':
		case '\r':
			comp->str.i += 1;
			continue;
		}
		uint32_t hash = 5381;
		int16_t s;
		word start = comp->str.i;
		while (comp->str.i < comp->str.size){
			c = comp->str.text[comp->str.i];
			if (c==SEP_CHAR){
				break;
			}
			s = comp->str.text[comp->str.i];
			hash = ((hash<<5)+hash)+s;
			comp->str.i += 1;
		}
		char copy = comp->str.text[comp->str.i];
		comp->str.text[comp->str.i] = '\0';
		REGISTER* r = REGISTER_map_access_by_hash(&comp->regmap, hash, (const char* const)comp->str.text+start);
		comp->str.text[comp->str.i] = copy;
		comp->str.i += 1;
		if (r == NULL){
			return 1;
		}
		return *r;
	}
	ASSERT_LOCAL(0, "Unexpected EOF\n");
}

word parse_number(compiler* const comp, byte max_bytes){
	char c = comp->str.text[comp->str.i];
	comp->str.i += 1;
	ASSERT_LOCAL(c=='0', "Expected numeric\n");
	c = comp->str.text[comp->str.i];
	comp->str.i += 1;
	ASSERT_LOCAL(c=='x' || c == 'X', "Expected numeric\n");
	word result = 0;
	word index = 0;
	byte max_chars = max_bytes*2;
	while (comp->str.i < comp->str.size){
		c = comp->str.text[comp->str.i];
		if (whitespace(c)){
			break;
		}
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
		else ASSERT_LOCAL(0, "Expected hex digit in numeric\n");
		result = (result << 4) | (converted & 0xF);
		comp->str.i += 1;
		index += 1;
	}
	ASSERT_LOCAL(index > 0 && index <= max_chars, "Too many bytes provided in numeric\n");
	comp->str.i += 1;
	return result;
}

uint16_t parse_short(compiler* const comp){
	return parse_number(comp, 2);
}

byte parse_byte(compiler* const comp){
	return parse_number(comp, 1);
}

byte parse_label(compiler* const comp){
	return 0; // TODO
}

#define WRITE_INSTRUCTION(li)\
	byte inst[] = li;\
	for (byte i = 0;i<4;++i){\
		comp->buf.text[comp->buf.i] = inst[i];\
		comp->buf.i += 1;\
	}

byte parse_full_register(compiler* const comp){
	byte r = parse_register(comp);
	byte p = parse_partition(comp);
	return REG(p, r);
}

#define PARSE_R(opc)\
	byte r = parse_full_register(comp);\
	WRITE_INSTRUCTION({opc(r)});
#define PARSE_S(opc)\
	uint16_t s = parse_short(comp);\
	WRITE_INSTRUCTION({opc(s)});
#define PARSE_B(opc)\
	byte b = parse_byte(comp);\
	WRITE_INSTRUCTION({opc(b)});
#define PARSE_RS(opc)\
	byte r = parse_full_register(comp);\
	uint16_t s = parse_short(comp);\
	WRITE_INSTRUCTION({opc(r, s)});
#define PARSE_RB(opc)\
	byte r = parse_full_register(comp);\
	byte b = parse_byte(comp);\
	WRITE_INSTRUCTION({opc(r, b)});
#define PARSE_RRR(opc)\
	byte a = parse_full_register(comp);\
	byte b = parse_full_register(comp);\
	byte c = parse_full_register(comp);\
	WRITE_INSTRUCTION({opc(a, b, c)});
#define PARSE_RR(opc)\
	byte b = parse_full_register(comp);\
	byte c = parse_full_register(comp);\
	WRITE_INSTRUCTION({opc(b, c)});
#define PARSE_RRB(opc)\
	byte a = parse_full_register(comp);\
	byte c = parse_full_register(comp);\
	byte b = parse_byte(comp);\
	WRITE_INSTRUCTION({opc(a, c, b)});
#define PARSE_BRANCH(opc)\
	if (comp->str.text[comp->str.i] == '0'){\
		int16_t s = parse_short(comp);\
		WRITE_INSTRUCTION({opc(1, s)});\
	}\
	else{\
		byte a = parse_full_register(comp);\
		WRITE_INSTRUCTION({opc(0, a)});\
	}


byte parse_opcode(compiler* const comp, OPCODE op){
	switch (op){
	case NOP: { WRITE_INSTRUCTION({NOP_}); } break;
	case LDS: { PARSE_RS(LDS_); } break;
	case LDB: { PARSE_RB(LDB_); } break;
	case LDA: { PARSE_RRR(LDA_); } break;
	case LDI: { PARSE_RRB(LDI_); } break;
	case STS: { PARSE_RS(STS_); } break;
	case STA: { PARSE_RRR(STA_); } break;
	case STB: { PARSE_RRB(STB_); } break;
	case MOV: { PARSE_RR(MOV_); } break;
	case SWP: { PARSE_RR(SWP_); } break;
	case ADS: { PARSE_RS(ADS_); } break;
	case SUS: { PARSE_RS(SUS_); } break;
	case MUS: { PARSE_RS(MUS_); } break;
	case DIS: { PARSE_RS(DIS_); } break;
	case MOS: { PARSE_RS(MOS_); } break;
	case ANS: { PARSE_RS(ANS_); } break;
	case ORS: { PARSE_RS(ORS_); } break;
	case SLS: { PARSE_RS(SLS_); } break;
	case SRS: { PARSE_RS(SRS_); } break;
	case XRS: { PARSE_RS(XRS_); } break;
	case ADI: { PARSE_RR(ADI_); } break;
	case SUI: { PARSE_RR(SUI_); } break;
	case MUI: { PARSE_RR(MUI_); } break;
	case DII: { PARSE_RR(DII_); } break;
	case MOI: { PARSE_RR(MOI_); } break;
	case ANI: { PARSE_RR(ANI_); } break;
	case ORI: { PARSE_RR(ORI_); } break;
	case SLI: { PARSE_RR(SLI_); } break;
	case SRI: { PARSE_RR(SRI_); } break;
	case XRI: { PARSE_RR(XRI_); } break;
	case ADD: { PARSE_RRR(ADD_); } break;
	case SUB: { PARSE_RRR(SUB_); } break;
	case MUL: { PARSE_RRR(MUL_); } break;
	case DIV: { PARSE_RRR(DIV_); } break;
	case MOD: { PARSE_RRR(MOD_); } break;
	case AND: { PARSE_RRR(AND_); } break;
	case OR: { PARSE_RRR(OR_); } break;
	case SHL: { PARSE_RRR(SHL_); } break;
	case SHR: { PARSE_RRR(SHR_); } break;
	case XOR: { PARSE_RRR(XOR_); } break;
	case INV: { PARSE_RR(INV_); } break;
	case COM: { PARSE_RR(COM_); } break;
	case INI: { PARSE_R(INI_); } break;
	case COI: { PARSE_R(COI_); } break;
	case CMP: { PARSE_RR(CMP_); } break;
	case CMS: { PARSE_RS(CMS_); } break;
	case RET: { PARSE_R(RET_); } break;
	case REI: { WRITE_INSTRUCTION({REI_}); } break;
	case RES: { PARSE_S(RES_); } break;
	case REB: { PARSE_B(REB_); } break;
	case ESC: { WRITE_INSTRUCTION({ESC_}); } break;
	case PSH: { PARSE_R(PSH_); } break;
	case PSS: { PARSE_S(PSS_); } break;
	case PSB: { PARSE_B(PSB_); } break;
	case POP: { PARSE_R(POP_); } break;
	case BNC: { PARSE_BRANCH(BNC_); } break;
	case BNE: { PARSE_BRANCH(BNE_); } break;
	case BEQ: { PARSE_BRANCH(BEQ_); } break;
	case BLT: { PARSE_BRANCH(BLT_); } break;
	case BGT: { PARSE_BRANCH(BGT_); } break;
	case BLE: { PARSE_BRANCH(BLE_); } break;
	case BGE: { PARSE_BRANCH(BGE_); } break;
	case JMP: { PARSE_BRANCH(JMP_); } break;
	case JNE: { PARSE_BRANCH(JNE_); } break;
	case JEQ: { PARSE_BRANCH(JEQ_); } break;
	case JLT: { PARSE_BRANCH(JLT_); } break;
	case JGT: { PARSE_BRANCH(JGT_); } break;
	case JLE: { PARSE_BRANCH(JLE_); } break;
	case JGE: { PARSE_BRANCH(JGE_); } break;
	case INT: { PARSE_B(INT_); } break;
	case INR: { PARSE_R(INR_); } break;
	default:
		ASSERT_LOCAL(0, "Unknown opcode\n");
	}
	return 0;
}

byte compile_cstr(compiler* const comp){
	while (comp->str.i < comp->str.size){
		char c = comp->str.text[comp->str.i];
		switch (c){
		case ' ':
		case '\n':
		case '\t':
		case '\r':
			comp->str.i += 1;
			continue;
		case '(':
			//TODO compound expression
			return 1;
		}
		uint32_t hash = 5381;
		int16_t s;
		word start = comp->str.i;
		while (comp->str.i < comp->str.size){
			c = comp->str.text[comp->str.i];
			if (whitespace(c)){
				break;
			}
			s = comp->str.text[comp->str.i];
			hash = ((hash<<5)+hash)+s;
			comp->str.i += 1;
		}
		char copy = comp->str.text[comp->str.i];
		comp->str.text[comp->str.i] = '\0';
		OPCODE* op = OPCODE_map_access_by_hash(&comp->opmap, hash, (const char* const)comp->str.text+start);
		comp->str.text[comp->str.i] = copy;
		comp->str.i += 1;
		if (op == NULL){
			//TODO label
			return 1;
		}
		parse_opcode(comp, *op);
		ASSERT_ERR(1);
	}
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
		snprintf(comp.err, ERROR_BUFFER, "at %s\n", comp.str.text+comp.str.i);
		fprintf(stderr, comp.err);
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
	reg[IP] = PROGRAM_START;
}

void flash_rom(byte* buffer, uint64_t size){
	for (uint64_t i = 0;i<size;++i){
		mem[PROGRAM_START+i] = buffer[i];
	}
}

void demo(){
	byte rom[] = {
		LDS_(REG(FULL, R7), 0x210),
		BNC_(0, REG(FULL, R7)),
		POP_(REG(L16, R2)),
		NOP_,
		LDS_(REG(FULL, R0), 0xCAFE),
		LDS_(REG(FULL, R1), 0xDEAD),
		LDB_(REG(LO, R2), 0xFE),
		LDB_(REG(HI, R2), 0xAF),
		LDS_(REG(L16, R3), 0xDEAD),
		LDS_(REG(LM16, R3), 0xBEEF),
		LDS_(REG(RM16, R3), 0xDEAF),
		LDS_(REG(R16, R3), 0xCAFE),
		LDS_(REG(HALF, R4), 0x1337),
		MOV_(REG(HALF, R4), REG(FULL, R3)),
		LDB_(REG(FULL, R6), 0x1),
		ADD_(REG(FULL, R5), REG(HALF, R2), REG(FULL, R6)),
		ADS_(REG(FULL, R5), 0x2),
		ADI_(REG(FULL, R6), REG(FULL, R6)),
		INI_(REG(FULL, R6)),
		COI_(REG(FULL, R6)),
		COM_(REG(HI, R6), REG(LO, R6)),
		INV_(REG(LO, R6), REG(HI, R6)),
		CMP_(REG(LO, R5), REG(LO, R6)),
		JNE_(1, -INSTRUCTION_WIDTH),
		JEQ_(1, INSTRUCTION_WIDTH*4),
		CMP_(REG(LO, R6), REG(HI, R6)),
		JLE_(1, -INSTRUCTION_WIDTH),
		JGT_(1, INSTRUCTION_WIDTH*4),
		CMS_(REG(R16, R2), 0xbaff),
		JGE_(1, -INSTRUCTION_WIDTH),
		JLT_(1, INSTRUCTION_WIDTH*-5),
		PSH_(REG(FULL, R3)),
		PSS_(0x1337),
		PSS_(0xC0DE),
		PSB_(0xAB),
		PSB_(0xCD),
		PSB_(0xEF),
		PSB_(0x64),
		POP_(REG(FULL, R0)),
		POP_(REG(FULL, R1)),
		LDS_(REG(FULL, R2), 688),
		BNC_(0, REG(FULL, R2)),
		POP_(REG(FULL, R3)),
		RES_(0xFACE),
		PSH_(REG(FULL, R6)),
		RET_(REG(FULL, R6)),
		NOP_, NOP_, NOP_, NOP_
	};
	byte cc[] = {
		LDS_(REG(L16, R3), 0xDEAD),
		LDS_(REG(LM16, R3), 0xBEEF),
		LDS_(REG(RM16, R3), 0xFACE),
		LDS_(REG(R16, R3), 0xCAFE),
		PSH_(REG(FULL, R3)),
		LDS_(REG(FULL, LR), 0x224),
		BNC_(0, REG(FULL, LR)),
		POP_(REG(FULL, R0)),
		NOP_,
		LDI_(REG(FULL, R1), REG(FULL, FP), 0x11),
		LDS_(REG(L16, R4), 0xBEEF),
		LDS_(REG(LM16, R4), 0xCAFE),
		LDS_(REG(RM16, R4), 0xDEAF),
		LDS_(REG(R16, R4), 0xFACE),
		XOR_(REG(FULL, R2), REG(FULL, R1), REG(FULL, R4)),
		STB_(REG(FULL, R2), REG(FULL, FP), 0x11),
		LDI_(REG(FULL, R1), REG(FULL, FP), 0x11),
		RET_(REG(FULL, R1)),
		NOP_, NOP_, NOP_, NOP_
	};
	setup_registers();
	flash_rom(cc, 1+(64*INSTRUCTION_WIDTH));
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

