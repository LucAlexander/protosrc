#include <stdio.h>
#include <inttypes.h>

#pragma GCC diagnostic ignored "-Wsequence-point"
enum {
	NOP,
	LDS, LDB, LDA, LDI,
	STS, STA, STB,
	MOV,
	ADS, SUS, MUS, DIS, MOS, ANS, ORS, SLS, SRS, XRS,
	ADI, SUI, MUI, DII, MOI, ANI, ORI, SLI, SRI, XRI,
	ADD, SUB, MUL, DIV, MOD, AND, OR, SHL, SHR, XOR,
	INV, COM, INI, COI,
	CMP, CMS,
	RET, REI, RES, REB,
	ESC,
	PSH, PSS, PSB,
	POP,
	BNC, BNE, BEQ, BLT, BGT, BLE, BGE,
	JMP, JNE, JEQ, JLT, JGT, JLE, JGE,
	INT, INR,
	OPCODE_COUNT
} OPCODE;

#define SHORT(lit) (lit&0xFF00)>>8, lit&0xFF
#define NOP_                   NOP, 0,    0, 0,
#define LDS_(dest, lit)        LDS, dest, SHORT(lit),
#define LDB_(dest, lit)        LDB, dest, lit, 0,
#define LDA_(dest, addr, off)  LDA, dest, addr, off,
#define LDI_(dest, addr, lit)  LDI, dest, addr, lit,
#define STS_(src, lit)         STS, src,  SHORT(lit),
#define STA_(src, addr, off)   STA, src,  addr, off,
#define STB_(src, addr, lit)   STB, src,  addr, lit,
#define MOV_(dst, src)         MOV, dst,  src, 0,
#define ADS_(dst, right)       ADS, dst,  SHORT(right),
#define SUS_(dst, right)       SUS, dst,  SHORT(right),
#define MUS_(dst, right)       MUS, dst,  SHORT(right),
#define DIS_(dst, right)       DIS, dst,  SHORT(right),
#define MOS_(dst, right)       MOS, dst,  SHORT(right),
#define ANS_(dst, right)       ANS, dst,  SHORT(right),
#define ORS_(dst, right)       ORS, dst,  SHORT(right),
#define SLS_(dst, right)       SLS, dst,  SHORT(right),
#define SRS_(dst, right)       SRS, dst,  SHORT(right),
#define XRS_(dst, right)       XRS, dst,  SHORT(right),
#define ADI_(dst, right)       ADI, dst,  right, 0,
#define SUI_(dst, right)       SUI, dst,  right, 0,
#define MUI_(dst, right)       MUI, dst,  right, 0,
#define DII_(dst, right)       DII, dst,  right, 0,
#define MOI_(dst, right)       MOI, dst,  right, 0,
#define ANI_(dst, right)       ANI, dst,  right, 0,
#define ORI_(dst, right)       ORI, dst,  right, 0,
#define SLI_(dst, right)       SLI, dst,  right, 0,
#define SRI_(dst, right)       SRI, dst,  right, 0,
#define XRI_(dst, right)       XRI, dst,  right, 0,
#define ADD_(dst, left, right) ADD, dst,  left, right,
#define SUB_(dst, left, right) SUB, dst,  left, right,
#define MUL_(dst, left, right) MUL, dst,  left, right,
#define DIV_(dst, left, right) DIV, dst,  left, right,
#define MOD_(dst, left, right) MOD, dst,  left, right,
#define AND_(dst, left, right) AND, dst,  left, right,
#define OR_(dst, left, right)  OR,  dst,  left, right,
#define SHL_(dst, left, right) SHL, dst,  left, right,
#define SHR_(dst, left, right) SHR, dst,  left, right,
#define XOR_(dst, left, right) XOR, dst,  left, right,
#define INV_(dst, src)         INV, dst,  src, 0,
#define COM_(dst, src)         COM, dst,  src, 0,
#define INI_(dst)              INI, dst,  0, 0,
#define COI_(dst)              COI, dst,  0, 0,
#define CMP_(left, right)      CMP, left, right, 0,
#define CMS_(left, right)      CMS, left, SHORT(right),
#define RET_(tar)              RET, tar,  0, 0,
#define REI_                   REI, 0,    0, 0,
#define RES_(tar)              RES, SHORT(tar), 0,
#define REB_(tar)              REB, tar,  0, 0,
#define ESC_                   ESC, 0,    0, 0,
#define PSH_(tar)              PSH, tar,  0, 0,
#define PSS_(tar)              PSS, SHORT(tar), 0,
#define PSB_(tar)              PSB, tar,  0, 0,
#define POP_(dst)              POP, dst,  0, 0,
#define BNC_(addr)             BNC, addr, 0, 0,
#define BNE_(addr)             BNE, addr, 0, 0,
#define BEQ_(addr)             BEQ, addr, 0, 0,
#define BLT_(addr)             BLT, addr, 0, 0,
#define BGT_(addr)             BGT, addr, 0, 0,
#define BLE_(addr)             BLE, addr, 0, 0,
#define BGE_(addr)             BGE, addr, 0, 0,
#define JMP_(off)              JMP, SHORT(off), 0,
#define JNE_(off)              JNE, SHORT(off), 0,
#define JEQ_(off)              JEQ, SHORT(off), 0,
#define JLT_(off)              JLT, SHORT(off), 0,
#define JGT_(off)              JGT, SHORT(off), 0,
#define JLE_(off)              JLE, SHORT(off), 0,
#define JGE_(off)              JGE, SHORT(off), 0,
#define INT_(b)                INT, b,    0, 0,
#define INR_(en)               INR, en,   0, 0,

enum {
	IP, SP, FP, SR,
	R0, R1, R2, R3, R4, R5, R6, R7,
	A0, A1, A2, A3,
	REGISTER_COUNT
} REGISTER;

enum {
	L16,
	LM16,
	RM16,
	R16,
	FULL,
	HALF,
	LO,
	HI
} REG_PARTITION;

#define REG(p, r) (p<<4 | r)

enum {
	ZERO=1,
	CARRY=2,
	SIGN=4,
	OVER=8
} STATUS_FLAG;

typedef uint64_t word;
typedef uint8_t byte;

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
	printf("\n");
	SHOW_REG(R0);
	SHOW_REG(R1);
	SHOW_REG(R2);
	SHOW_REG(R3);
	SHOW_REG(R4);
	SHOW_REG(R5);
	SHOW_REG(R6);
	SHOW_REG(R7);
	printf("\n");
	SHOW_REG(A0);
	SHOW_REG(A1);
	SHOW_REG(A2);
	SHOW_REG(A3);
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
     
#define NEXT (mem[++reg[IP]])
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
	reg[IP] += 1;

#define ALU_I(operator)\
	byte dst = NEXT;\
	byte src = NEXT;\
	word left; ACCESS_REG(left, dst);\
	word right; ACCESS_REG(right, src);\
	LOAD_REG(dst, left operator right);\
	reg[IP] += 2;

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
	reg[IP] += 3;

#define ALU_U(operator)\
	byte tar = NEXT;\
	byte src = NEXT;\
	word val; ACCESS_REG(val, src);\
	LOAD_REG(tar, operator val);\
	reg[IP] += 2;

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
	byte adr = NEXT;\
	ACCESS_REG(reg[IP], adr);

void interpret(){
	while (1){
		show_machine();
		getc(stdin);
		byte op = mem[reg[IP]];
		switch (op){
			case NOP: { if (1) {return;} else {reg[IP] += INSTRUCTION_WIDTH;}} break;
			case LDS: { byte b = NEXT; LOAD_REG(b, SHORT_LITERAL); reg[IP] += 1; } break;
			case LDB: { byte b = NEXT; LOAD_REG(b, NEXT); reg[IP] += 2; } break;
			case LDA: {
					byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
					word a; ACCESS_REG(a, adr);
					word o; ACCESS_REG(o, off);
					LOAD_REG_ADDR(dst, a+o); reg[IP] += 1;
				} break;
			case LDI: {
					byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
					word a; ACCESS_REG(a, adr);
					LOAD_REG_ADDR(dst, a+off); reg[IP] += 1;
				} break;
			case STS: { byte src = NEXT; uint16_t adr = SHORT_LITERAL; STORE_REG(adr, src); reg[IP] += 1; } break;
			case STA: {
					byte src = NEXT; byte adr = NEXT; byte off = NEXT;
					word a; ACCESS_REG(a, adr);
					word o; ACCESS_REG(o, off);
					STORE_REG(a+o, src); reg[IP] += 1;
				} break;
			case STB: {
					byte src = NEXT; byte adr = NEXT; byte off = NEXT;
					word a; ACCESS_REG(a, adr);
					STORE_REG(a+off, src); reg[IP] += 1;
				} break;
			case MOV: {
					byte dst = NEXT; byte src = NEXT;
					word s; ACCESS_REG(s, src);
					LOAD_REG(dst, s); reg[IP] += 2;
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
					reg[IP] += 2;
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
			case PSH: { byte tar = NEXT; PUSH_REG(tar); reg[IP] += 3; } break;
			case PSS: {
					uint16_t lit = SHORT_LITERAL;
					reg[SP] -= 1;
					*(uint16_t*)(&mem[reg[SP]]) = lit;
					reg[SP] -= 1;
					reg[IP] += 2;
				} break;
			case PSB: {
					byte b = NEXT;
					*(byte*)(&mem[reg[SP]]) = b;
					reg[SP] -= 1;
					reg[IP] += 3;
				} break;
			case POP: { byte tar = NEXT; POP_REG(tar); reg[IP] += 3; } break;
			case BNC: { BRANCH_LINK; } break;
			case BNE: {
					if (((reg[SR] & ZERO) == 0) || ((reg[SR] & CARRY) != 0))
					{ BRANCH_LINK; } else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case BEQ: {
					if (((reg[SR] & ZERO) != 0) && ((reg[SR] & CARRY) == 0))
					{ BRANCH_LINK; } else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case BLT: {
					if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) != 0))
					{ BRANCH_LINK; } else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case BGT: {
					if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) == 0))
					{ BRANCH_LINK; } else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case BLE: {
					if ((reg[SR] & ZERO) != (reg[SR] & CARRY))
					{ BRANCH_LINK; } else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case BGE: {
					if ((reg[SR] & CARRY) == 0)
					{ BRANCH_LINK; } else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case JMP: { int16_t off = SHORT_LITERAL; reg[IP] += (off-2); } break;
			case JNE: {
					if (((reg[SR] & ZERO) == 0) || ((reg[SR] & CARRY) != 0))
					{ int16_t off = SHORT_LITERAL; reg[IP] += (off-2); }
					else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case JEQ: {
					if (((reg[SR] & ZERO) != 0) && ((reg[SR] & CARRY) == 0))
					{ int16_t off = SHORT_LITERAL; reg[IP] += (off-2); }
					else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case JLT: {
					if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) != 0))
					{ int16_t off = SHORT_LITERAL; reg[IP] += (off-2); }
					else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case JGT: {
					if (((reg[SR] & ZERO) == 0) && ((reg[SR] & CARRY) == 0))
					{ int16_t off = SHORT_LITERAL; reg[IP] += (off-2); }
					else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case JLE: {
					if ((reg[SR] & ZERO) != (reg[SR] & CARRY))
					{ int16_t off = SHORT_LITERAL; reg[IP] += (off-2); }
					else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case JGE: {
					if ((reg[SR] & CARRY) == 0)
					{ int16_t off = SHORT_LITERAL; reg[IP] += (off-2); }
					else { reg[IP] += INSTRUCTION_WIDTH; }
				} break;
			case INT: { reg[IP] += INSTRUCTION_WIDTH; } break;
			case INR: { reg[IP] += INSTRUCTION_WIDTH; } break;
			default:
				printf("Uknown upcode\n");
				return;
		}
	}
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

int32_t main(int argc, char** argv){
	byte rom[] = {
		LDS_(REG(FULL, R7), 0x210)
		BNC_(REG(FULL, R7))
		POP_(REG(L16, R2))
		NOP_
		LDS_(REG(FULL, R0), 0xCAFE)
		LDS_(REG(FULL, R1), 0xDEAD)
		LDB_(REG(LO, R2), 0xFE)
		LDB_(REG(HI, R2), 0xAF)
		LDS_(REG(L16, R3), 0xDEAD)
		LDS_(REG(LM16, R3), 0xBEEF)
		LDS_(REG(RM16, R3), 0xDEAF)
		LDS_(REG(R16, R3), 0xCAFE)
		LDS_(REG(HALF, R4), 0x1337)
		MOV_(REG(HALF, R4), REG(FULL, R3))
		LDB_(REG(FULL, R6), 0x1)
		ADD_(REG(FULL, R5), REG(HALF, R2), REG(FULL, R6))
		ADS_(REG(FULL, R5), 0x2)
		ADI_(REG(FULL, R6), REG(FULL, R6))
		INI_(REG(FULL, R6))
		COI_(REG(FULL, R6))
		COM_(REG(HI, R6), REG(LO, R6))
		INV_(REG(LO, R6), REG(HI, R6))
		CMP_(REG(LO, R5), REG(LO, R6))
		JNE_(-INSTRUCTION_WIDTH)
		JEQ_(INSTRUCTION_WIDTH*4)
		CMP_(REG(LO, R6), REG(HI, R6))
		JLE_(-INSTRUCTION_WIDTH)
		JGT_(INSTRUCTION_WIDTH*4)
		CMS_(REG(R16, R2), 0xbaff)
		JGE_(-INSTRUCTION_WIDTH)
		JLT_(INSTRUCTION_WIDTH*-5)
		PSH_(REG(FULL, R3))
		PSS_(0x1337)
		PSS_(0xC0DE)
		PSB_(0xAB)
		PSB_(0xCD)
		PSB_(0xEF)
		PSB_(0x64)
		POP_(REG(FULL, R0))
		POP_(REG(FULL, R1))
		LDS_(REG(FULL, R2), 688)
		BNC_(REG(FULL, R2))
		POP_(REG(FULL, R3))
		RES_(0xFACE)
		PSH_(REG(FULL, R6))
		RET_(REG(FULL, R6))
		NOP_ NOP_ NOP_ NOP_
	};
	setup_registers();
	flash_rom(rom, 1+(64*INSTRUCTION_WIDTH));
	interpret();
	return 0;
}
