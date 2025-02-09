#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "orb.h"
#include "hashmap.h"

#include <SDL2/SDL.h>

#pragma GCC diagnostic ignored "-Wsequence-point"

MAP_IMPL(OPCODE)
MAP_IMPL(REGISTER)
MAP_IMPL(REG_PARTITION)
MAP_IMPL(EXTERNAL_CALLS)
MAP_IMPL(block_scope)
MAP_IMPL(loc_thunk)
MAP_IMPL(char)
MAP_IMPL(word)
MAP_IMPL(macro)

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
#define EXT_(b)                EXT, b,    0, 0
#define EXR_(en)               EXR, en,   0, 0

#define REG(p, r) (p<<4 | r)

#define SHOW_REG(mach, r)\
	printf("                                   %s: %016lx (%lu) (%ld)\n\033[0m\033[1m", #r, mach->reg[r], mach->reg[r], mach->reg[r]);

void
show_keyboard(machine* const mach){
	printf("                                                                             \033[4mKB Device\033[0m\n");
	for (word i = 0;i<256;++i){
		printf("                                                                               ");
		byte k = i+16;
		for (;i<k;++i){
			byte key = mach->dev[KEYBOARD_DEVICE][i];
			if (key == 1){
				printf("x");
			}
			else{
				printf(" ");
			}
		}
		printf("\n");
	}
}

void
show_registers(machine* const mach){
	printf("                                   \033[4mCPU Registers\033[0m\n");
	printf("\033[1;32m");
	SHOW_REG(mach, IP);
	printf("\033[1;33m");
	SHOW_REG(mach, SP);
	SHOW_REG(mach, CR);
	printf("\033[1;31m");
	SHOW_REG(mach, FP);
	SHOW_REG(mach, SR);
	SHOW_REG(mach, LR);
	SHOW_REG(mach, AR);
printf("\n");
	SHOW_REG(mach, R0);
	SHOW_REG(mach, R1);
	SHOW_REG(mach, R2);
	SHOW_REG(mach, R3);
	SHOW_REG(mach, R4);
	SHOW_REG(mach, R5);
	SHOW_REG(mach, R6);
	SHOW_REG(mach, R7);
	SHOW_REG(mach, R8);
	printf("\n");
}

void
show_mem(machine* const mach){
	printf("\033[4mProgram / Stack\033[0m\n");
	printf("\033[1;32m");
	for (byte i = 0;i<INSTRUCTION_WIDTH*8;){
		word address = PROGRAM_START+(INSTRUCTION_WIDTH*mach->reg[IP]);
		printf("%016lx | ", address+i);
		if (address+i > MEMORY_SIZE){
			break;
		}
		byte m = i+INSTRUCTION_WIDTH;
		if ((address+m-1) > MEMORY_SIZE){
			m = MEMORY_SIZE-address;
		}
		for (;i<m;++i){
			printf("%02x \033[0m\033[1m", mach->mem[address+i]);
		}
		printf("\033[0m\033[1m\n");
	}
	printf("...              | ...\n");
	word left = mach->reg[SP];
	while ((left--) % 4 != 1){}
	word bottom = MEMORY_SIZE;
	if (mach->reg[SP] < bottom-INSTRUCTION_WIDTH*24){
		bottom = mach->reg[SP] + INSTRUCTION_WIDTH*24;
	}
	for (word i = left;i<bottom;){
		word m = i+INSTRUCTION_WIDTH;
		printf("%016lx | ", i);
		for (;i<m;++i){
			if (i == mach->reg[SP]){
				printf("\033[1;33m");
			}
			else if (i == mach->reg[FP]){
				printf("\033[1;31m");
			}
			printf("%02x \033[0m\033[1m", mach->mem[i]);
		}
		printf("\033[0m\033[1m\n");
	}
}

void
show_machine(machine* const mach){
	printf("\033[2J");
	printf("\033[H\033[1m");
	show_keyboard(mach);
	printf("\033[H\033[1m");
	show_registers(mach);
	printf("\033[H\033[1m");
	show_mem(mach);
}
     
#define NEXT (mach->mem[++ip])
#define SHORT_LITERAL ((NEXT<<8)+(NEXT))
#define LOAD_REG(b, v)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		mach->reg[r] = v;\
		break;\
	case HALF:\
		*mach->half[r] = v;\
		break;\
	case LO:\
		*mach->lo[r] = v;\
		break;\
	case HI:\
		*mach->hi[r] = v;\
		break;\
	default:\
		*mach->quar[(r*4)+partition] = v;\
		break;\
	}\
}

#define LOAD_REG_ADDR(b, v)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		mach->reg[r] = *(word*)(&mach->mem[v]);\
		break;\
	case HALF:\
		*mach->half[r] = *(uint32_t*)(&mach->mem[v]);\
		break;\
	case LO:\
		*mach->lo[r] = mach->mem[v];\
		break;\
	case HI:\
		*mach->hi[r] = mach->mem[v];\
		break;\
	default:\
		*mach->quar[(r*4)+partition] = *(uint16_t*)(&mach->mem[v]);\
		break;\
	}\
}

#define ACCESS_REG(v, b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		v = mach->reg[r];\
		break;\
	case HALF:\
		v = *mach->half[r];\
		break;\
	case LO:\
		v = *mach->lo[r];\
		break;\
	case HI:\
		v = *mach->hi[r];\
		break;\
	default:\
		v = *mach->quar[(r*4)+partition];\
		break;\
	}\
}

#define STORE_REG(a, b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch (partition){\
	case FULL:\
		*(word*)(&mach->mem[a]) = mach->reg[r];\
		break;\
	case HALF:\
		*(uint32_t*)(&mach->mem[a]) = *mach->half[r];\
		break;\
	case LO:\
		*(byte*)(&mach->mem[a]) = *mach->lo[r];\
		break;\
	case HI:\
		*(byte*)(&mach->mem[a]) = *mach->hi[r];\
		break;\
	default:\
		*(uint16_t*)(&mach->mem[a]) = *mach->quar[(r*4)+partition];\
		break;\
	}\
}

#define PUSH_REG(b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch(partition){\
	case FULL:\
		mach->reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mach->mem[mach->reg[SP]]) = mach->reg[r];\
		mach->reg[SP] -= 1;\
		break;\
	case HALF:\
		mach->reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mach->mem[mach->reg[SP]]) = (word)(*mach->half[r]);\
		mach->reg[SP] -= 1;\
		break;\
	case LO:\
		mach->reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mach->mem[mach->reg[SP]]) = (word)(*mach->lo[r]);\
		mach->reg[SP] -= 1;\
		break;\
	case HI:\
		mach->reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mach->mem[mach->reg[SP]]) = (word)(*mach->hi[r]);\
		mach->reg[SP] -= 1;\
		break;\
	default:\
		mach->reg[SP] -= (sizeof(word)-1);\
		*(word*)(&mach->mem[mach->reg[SP]]) = (word)(*mach->quar[(r*4)+partition]);\
		mach->reg[SP] -= 1;\
		break;\
	}\
}

#define POP_REG(b)\
{\
	byte r = b&0xF;\
	byte partition = (b&0x70) >> 4;\
	switch(partition){\
	case FULL:\
		mach->reg[SP] += 1;\
		mach->reg[r] = *(word*)(&mach->mem[mach->reg[SP]]);\
		mach->reg[SP] += sizeof(word)-1;\
		break;\
	case HALF:\
		mach->reg[SP] += 1;\
		*mach->half[r] = *(word*)(&mach->mem[mach->reg[SP]]);\
		mach->reg[SP] += sizeof(word)-1;\
		break;\
	case LO:\
		mach->reg[SP] += 1;\
		*mach->lo[r] = *(word*)(&mach->mem[mach->reg[SP]]);\
		mach->reg[SP] += sizeof(word)-1;\
		break;\
	case HI:\
		mach->reg[SP] += 1;\
		*mach->hi[r] = *(word*)(&mach->mem[mach->reg[SP]]);\
		mach->reg[SP] += sizeof(word)-1;\
		break;\
	default:\
		mach->reg[SP] += 1;\
		*mach->quar[(r*4)+partition] = *(word*)(&mach->mem[mach->reg[SP]]);\
		mach->reg[SP] += sizeof(word)-1;\
		break;\
	}\
}

#define ALU_S(operator)\
	byte dst = NEXT;\
	uint16_t right = SHORT_LITERAL;\
	word left; ACCESS_REG(left, dst);\
	LOAD_REG(dst, left operator right);\
	mach->reg[IP] += 1;

#define ALU_I(operator)\
	byte dst = NEXT;\
	byte src = NEXT;\
	word left; ACCESS_REG(left, dst);\
	word right; ACCESS_REG(right, src);\
	LOAD_REG(dst, left operator right);\
	mach->reg[IP] += 1;

#define ALU(operator)\
	byte dst = NEXT;\
	byte src_a = NEXT;\
	byte src_b = NEXT;\
	word left; ACCESS_REG(left, src_a);\
	word right; ACCESS_REG(right, src_b);\
	LOAD_REG(dst, left operator right);\
	mach->reg[IP] += 1;

#define ALU_UI(operator)\
	byte tar = NEXT;\
	word val; ACCESS_REG(val, tar);\
	LOAD_REG(tar, operator val);\
	mach->reg[IP] += 1;

#define ALU_U(operator)\
	byte tar = NEXT;\
	byte src = NEXT;\
	word val; ACCESS_REG(val, src);\
	LOAD_REG(tar, operator val);\
	mach->reg[IP] += 1;

#define COMPARE_FLAGS(a, b)\
	if (a<b){\
		mach->reg[SR] |= CARRY; \
		mach->reg[SR] &= ~ZERO;\
	}\
	else if (a==b){\
		mach->reg[SR] |= ZERO; \
		mach->reg[SR] &= ~CARRY;\
	}\
	else{\
		mach->reg[SR] &= ~ZERO; \
		mach->reg[SR] &= ~CARRY;\
	}

#define BRANCH_LINK\
	PUSH_REG(REG(FULL, IP));\
	PUSH_REG(REG(FULL, FP));\
	PUSH_REG(REG(FULL, LR));\
	PUSH_REG(REG(FULL, CR));\
	PUSH_REG(REG(FULL, AR));\
	mach->reg[FP] = mach->reg[SP];\
	ip += 1;\
	byte adr = NEXT;\
	word val; ACCESS_REG(val, adr);\
	mach->reg[IP] += *(int64_t*)(&val);

#define BRANCH_LINK_TO\
	PUSH_REG(REG(FULL, IP));\
	PUSH_REG(REG(FULL, FP));\
	PUSH_REG(REG(FULL, LR));\
	PUSH_REG(REG(FULL, CR));\
	PUSH_REG(REG(FULL, AR));\
	mach->reg[FP] = mach->reg[SP];\
	mach->reg[IP] = mach->reg[LR]-PROGRAM_START;\
	mach->reg[IP] /= 4;

#define BRANCH_JUMP\
	PUSH_REG(REG(FULL, IP));\
	PUSH_REG(REG(FULL, FP));\
	PUSH_REG(REG(FULL, LR));\
	PUSH_REG(REG(FULL, CR));\
	PUSH_REG(REG(FULL, AR));\
	mach->reg[FP] = mach->reg[SP];\
	int16_t offset = SHORT_LITERAL;\
	mach->reg[IP] += offset;

#define JUMP_REG\
	ip += 1;\
	byte adr = NEXT;\
	word val; ACCESS_REG(val, adr);\
	mach->reg[IP] += *(int64_t*)(&val);

#define JUMP_REG_TO\
	mach->reg[IP] = mach->reg[LR];\

#define JUMP\
	int16_t offset = SHORT_LITERAL;\
	mach->reg[IP] += offset;

void
interpret(machine* const mach, byte debug){
	SDL_Texture* texture = SDL_CreateTexture(
        mach->renderer,
        SDL_PIXELFORMAT_RGBA32,
        SDL_TEXTUREACCESS_STREAMING,
        SCREEN_WIDTH,
        SCREEN_HEIGHT
	);
	word clock = 0;
	while (1){
		if (clock++ % 10000 == 0){
			poll_input(mach);
			SDL_UpdateTexture(texture, NULL, mach->frame_buffer, SCREEN_WIDTH*sizeof(uint32_t));
			SDL_RenderCopy(mach->renderer, texture, NULL, NULL);
			SDL_RenderPresent(mach->renderer);
		}
		if (debug){
			show_machine(mach);
			getc(stdin);
		}
		word ip = PROGRAM_START+(mach->reg[IP]*INSTRUCTION_WIDTH);
		byte op = mach->mem[ip];
		switch (op){
		case NOP: { mach->reg[IP] += 1; } break;
		case LDS: { byte b = NEXT; LOAD_REG(b, SHORT_LITERAL); mach->reg[IP] += 1; } break;
		case LDA: {
				byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				word o; ACCESS_REG(o, off);
				LOAD_REG_ADDR(dst, a+o);
				mach->reg[IP] += 1;
			} break;
		case LDI: {
				byte dst = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				LOAD_REG_ADDR(dst, a+off);
				mach->reg[IP] += 1;
			} break;
		case STS: { byte src = NEXT; uint16_t adr = SHORT_LITERAL; STORE_REG(adr, src); mach->reg[IP] += 1; } break;
		case STA: {
				byte src = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				word o; ACCESS_REG(o, off);
				STORE_REG(a+o, src);
				mach->reg[IP] += 1;
			} break;
		case STB: {
				byte src = NEXT; byte adr = NEXT; byte off = NEXT;
				word a; ACCESS_REG(a, adr);
				STORE_REG(a+off, src);
				mach->reg[IP] += 1;
			} break;
		case MOV: {
				byte dst = NEXT; byte src = NEXT;
				word s; ACCESS_REG(s, src);
				LOAD_REG(dst, s);
				mach->reg[IP] += 1;
			} break;
		case SWP: {
				byte a = NEXT; byte b = NEXT;
				word s; ACCESS_REG(s, a);
				word d; ACCESS_REG(d, b);
				LOAD_REG(a, d); LOAD_REG(b, s);
				mach->reg[IP] += 1;
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
				mach->reg[IP] += 1;
			} break;
		case CMS: {
				byte a = NEXT; uint16_t right = SHORT_LITERAL;
				word left; ACCESS_REG(left, a);
				COMPARE_FLAGS(left, right);
				mach->reg[IP] += 1;
			} break;
		case RET: {
				byte tar = NEXT;
				mach->reg[SP] = mach->reg[FP];
				word preserve; ACCESS_REG(preserve, tar);
				POP_REG(REG(FULL,AR));
				POP_REG(REG(FULL,CR));
				POP_REG(REG(FULL,LR));
				POP_REG(REG(FULL,FP));
				POP_REG(REG(FULL,IP));
				mach->reg[SP] = mach->reg[CR];
				POP_REG(REG(FULL,CR));
				word temp = mach->reg[AR];
				mach->reg[AR] = preserve;
				PUSH_REG(REG(FULL, AR));
				mach->reg[AR] = temp;
				mach->reg[IP] += 1;
			} break;
		case REI: {
				mach->reg[SP] = mach->reg[FP];
				POP_REG(REG(FULL,AR));
				POP_REG(REG(FULL,CR));
				POP_REG(REG(FULL,LR));
				POP_REG(REG(FULL,FP));
				POP_REG(REG(FULL,IP));
				mach->reg[SP] = mach->reg[CR];
				POP_REG(REG(FULL,CR));
				mach->reg[IP] += 1;
			} break;
		case RES: {
				uint16_t tar = SHORT_LITERAL;
				mach->reg[SP] = mach->reg[FP];
				POP_REG(REG(FULL,AR));
				POP_REG(REG(FULL,CR));
				POP_REG(REG(FULL,LR));
				POP_REG(REG(FULL,FP))
				POP_REG(REG(FULL,IP))
				mach->reg[SP] = mach->reg[CR];
				POP_REG(REG(FULL,CR));
				mach->reg[SP] -= (sizeof(word)-1);
				*(word*)(&mach->mem[mach->reg[SP]]) = tar;
				mach->reg[SP] -= 1;
				mach->reg[IP] += 1;
			} break;
		case CAL: {
				PUSH_REG(REG(FULL,CR));
				mach->reg[CR] = mach->reg[SP];
				mach->reg[IP] += 1;
			} break;
		case PSH: { byte tar = NEXT; PUSH_REG(tar); mach->reg[IP] += 1; } break;
		case PSS: {
				uint16_t lit = SHORT_LITERAL;
				mach->reg[SP] -= (sizeof(word)-1);
				*(word*)(&mach->mem[mach->reg[SP]]) = lit;
				mach->reg[SP] -= 1;
				mach->reg[IP] += 1;
			} break;
		case POP: { byte tar = NEXT; POP_REG(tar); mach->reg[IP] += 1; } break;
		case BNC: {
				byte mode = NEXT;
				if (mode == 0) { BRANCH_LINK; }
				else if (mode == 1) { BRANCH_JUMP; }
				else { BRANCH_LINK_TO; }
			} break;
		case BNE: {
				if (((mach->reg[SR] & ZERO) == 0) || ((mach->reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case BEQ: {
				if (((mach->reg[SR] & ZERO) != 0) && ((mach->reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case BLT: {
				if (((mach->reg[SR] & ZERO) == 0) && ((mach->reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case BGT: {
				if (((mach->reg[SR] & ZERO) == 0) && ((mach->reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case BLE: {
				if ((mach->reg[SR] & ZERO) != (mach->reg[SR] & CARRY)){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case BGE: {
				if ((mach->reg[SR] & CARRY) == 0){
					byte mode = NEXT;
					if (mode == 0) { BRANCH_LINK; }
					else if (mode == 1) { BRANCH_JUMP; }
					else { BRANCH_LINK_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case JMP: {
				byte mode = NEXT;
				if (mode == 0) { JUMP_REG; } else if (mode == 1) { JUMP; } else { JUMP_REG_TO; }
			} break;
		case JNE: {
				if (((mach->reg[SR] & ZERO) == 0) || ((mach->reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case JEQ: {
				if (((mach->reg[SR] & ZERO) != 0) && ((mach->reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case JLT: {
				if (((mach->reg[SR] & ZERO) == 0) && ((mach->reg[SR] & CARRY) != 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case JGT: {
				if (((mach->reg[SR] & ZERO) == 0) && ((mach->reg[SR] & CARRY) == 0)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case JLE: {
				if ((mach->reg[SR] & ZERO) != (mach->reg[SR] & CARRY)){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case JGE: {
				if ((mach->reg[SR] & CARRY) == 0){
					byte mode = NEXT;
					if (mode == 0) { JUMP_REG; }
					else if (mode == 1) { JUMP; }
					else { JUMP_REG_TO; }
				}
				else { mach->reg[IP] += 1; }
			} break;
		case EXT: {
				byte a = NEXT;
				if (interpret_external(mach, a) == 0){
					return;
				}
				mach->reg[IP] += 1;
			} break;
		case EXR: {
				byte adr = NEXT;
				word a; ACCESS_REG(a, adr);
				if (interpret_external(mach, a) == 0){
					return;
				}
				mach->reg[IP] += 1;
			} break;
		default:
			printf("Unknown upcode\n");
			return;
		}
	}
}

byte
interpret_external(machine* const mach, byte ext){
	switch (ext){
	case EXT_OUT:
		word str = mach->reg[R0];
		word len = mach->reg[R1];
		byte* data = &mach->mem[str];
		byte save = data[len];
		data[len] = '\0';
		printf("%s",(const char*)data);
		data[len] = save;
		return 1;
	case EXT_END:
		return 0;
	case EXT_MEM:
		word mem_len = mach->reg[R0];
		if (mem_len + mach->mem_ptr < MEMORY_SIZE){
			mach->reg[R0] = mach->mem_ptr;
			mach->mem_ptr += mem_len;
			return 1;
		}
		return 0;
	case EXT_MEM_PROG:
		word prog_len = mach->reg[R0];
		if (prog_len + mach->prog_ptr < AUX_MEM_START){
			mach->reg[R0] = mach->prog_ptr;
			mach->prog_ptr += prog_len;
			return 1;
		}
		return 0;
	case EXT_MEM_AUX:
		word aux_len = mach->reg[R0];
		if (aux_len + mach->aux_ptr < FULL_MEM_SIZE){
			mach->reg[R0] = mach->aux_ptr;
			mach->aux_ptr += aux_len;
			return 1;
		}
		return 0;
	case EXT_KEY:
		byte k = mach->reg[R0];
		mach->reg[R0] = mach->dev[KEYBOARD_DEVICE][k];
		return 1;
	case EXT_SCR_STAT:
		mach->reg[R0] = SCREEN_WIDTH;
		mach->reg[R1] = SCREEN_HEIGHT;
		mach->reg[R2] = 4;
		return 1;
	case EXT_SCR_DRAW:
		mach->frame_buffer[mach->reg[R0]] = (uint32_t)(mach->reg[R1] & 0xFFFFFFFF);
		return 1;
	default:
		fprintf(stderr, "External call unimplemented");
		return 0;
	}
	return 0;
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

void
setup_opcode_map(OPCODE_map* opmap){
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
	OPCODE_map_insert(opmap, "EXT", ops++);
	OPCODE_map_insert(opmap, "EXR", ops++);
}

void
setup_register_map(REGISTER_map* regmap){
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
	REGISTER_map_insert(regmap, "AR", regs++);
	REGISTER_map_insert(regmap, "rA", regs++);
	REGISTER_map_insert(regmap, "rB", regs++);
	REGISTER_map_insert(regmap, "rC", regs++);
	REGISTER_map_insert(regmap, "rD", regs++);
	REGISTER_map_insert(regmap, "rE", regs++);
	REGISTER_map_insert(regmap, "rF", regs++);
	REGISTER_map_insert(regmap, "rG", regs++);
	REGISTER_map_insert(regmap, "rH", regs++);
	REGISTER_map_insert(regmap, "rI", regs++);
}

void
setup_partition_map(REG_PARTITION_map* partmap){
	REG_PARTITION* parts = pool_request(partmap->mem, sizeof(REG_PARTITION)*PARTITION_COUNT);
	for (REG_PARTITION i = 0;i<PARTITION_COUNT;++i){
		parts[i] = i;
	}
	REG_PARTITION_map_insert(partmap, "Z", parts++);
	REG_PARTITION_map_insert(partmap, "Y", parts++);
	REG_PARTITION_map_insert(partmap, "X", parts++);
	REG_PARTITION_map_insert(partmap, "W", parts++);
	REG_PARTITION_map_insert(partmap, "Q", parts++);
	REG_PARTITION_map_insert(partmap, "D", parts++);
	REG_PARTITION_map_insert(partmap, "LO", parts++);
	REG_PARTITION_map_insert(partmap, "HI", parts++);
}

void
setup_external_call_map(EXTERNAL_CALLS_map* extmap){
	EXTERNAL_CALLS* ext = pool_request(extmap->mem, sizeof(EXTERNAL_CALLS)*EXT_COUNT);
	for (EXTERNAL_CALLS i = 0;i<EXT_COUNT;++i){
		ext[i] = i;
	}
	EXTERNAL_CALLS_map_insert(extmap, "OUT", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "END", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "MEM", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "MEM_PROG", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "MEM_AUX", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "KEY", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "SCR_STAT", ext++);
	EXTERNAL_CALLS_map_insert(extmap, "SCR_DRAW", ext++);
}

byte
whitespace(char c){
	return (c=='\n' || c==' ' || c=='\r' || c == '\t');
}

#define LEXERR " [Line %lu] \033[1mLexing Error\033[0m "
#define PARSERR " \033[1mParsing Error\033[0m "
#define PARSERRFIX " at \033[1;31m %s\033[0m\n"

#define ENTRYPOINT_CODE "(main) EXT END"

void
nest_lex_cstr(compiler* const comp, char* text, word size){
	compiler nested = {
		.str.size = size,
		.str.i = 0,
		.str.text = text,
		.tokens = comp->tokens,
		.token_count = comp->token_count,
		.opmap = comp->opmap,
		.regmap = comp->regmap,
		.partmap = comp->partmap,
		.extmap = comp->extmap,
		.inclusions = comp->inclusions,
		.mem = comp->mem,
		.code = comp->code,
		.tok = comp->tok,
		.buf = NULL,
		.err = comp->err
	};
	lex_cstr(&nested, 1, 1);
	comp->token_count = nested.token_count;
}

void
impute_entrypoint(compiler* const comp){
	word size = strlen(ENTRYPOINT_CODE);
	char* text = pool_request(comp->mem, size);
	strcpy(text, ENTRYPOINT_CODE);
	nest_lex_cstr(comp, text, size);
}

byte
issymbol(char c){
	return (
		(c > ' ' && c < '0') ||
		(c > ';' && c < 'A') ||
		(c > '[' && c < '_') ||
		(c == '~') ||
		(c == '|')
	);
}

void
lex_symbol(compiler* const comp, token* t){
	t->type = SYMBOL_TOKEN;
	while (comp->str.i < comp->str.size){
		char c = comp->str.text[comp->str.i];
		if (issymbol(c)){
			t->size += 1;
		}
		comp->str.i += 1;
		return;
	}
	return;
}

byte
lex_cstr(compiler* const comp, byte nested, byte noentry){
	if (nested == 0){
		comp->tokens = pool_request(comp->tok, sizeof(token));
		comp->token_count = 0;
		if (noentry == 0){
			impute_entrypoint(comp);
			ASSERT_ERR(0);
		}
	}
	word line = 1;
	byte include = 0;
	token* t = NULL;
	while (comp->str.i < comp->str.size){
		if (t != NULL){
			if (include == 1){
			   	ASSERT_LOCAL(t->type == IDENTIFIER_TOKEN, LEXERR " Expected name of included source file\n", line);
				include = 0;
				comp->token_count -= 2;
				char* filename = pool_request(comp->mem, t->size+5);
				strncpy(filename, t->text, t->size);
				strcat(filename, ".src");
				filename[t->size+4] = '\0';
				byte dup = char_map_insert(&comp->inclusions, filename, filename);
				ASSERT_LOCAL(dup == 0, LEXERR " Duplicate inclusion '%s'\n", line, filename);
				FILE* inc = fopen(filename, "r");
				ASSERT_LOCAL(inc != NULL, LEXERR " Could not find file '%s'\n", line, filename);
				char* inc_text = comp->mem->ptr;
				word read_bytes = fread(inc_text, sizeof(byte), comp->mem->left, inc);
				fclose(inc);
				ASSERT_LOCAL(read_bytes < comp->mem->left, LEXERR " Included file '%s' too big\n", line, filename);
				pool_request(comp->mem, read_bytes);
				nest_lex_cstr(comp, inc_text, read_bytes);	
				ASSERT_ERR(0);
			}
			else if (include == 0 && t->type == INCLUDE_TOKEN){
				include = 1;
			}
		}
		t = &comp->tokens[comp->token_count];
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
		case OPEN_MACRO_TOKEN:
		case CLOSE_MACRO_TOKEN:
		case EVAL_MACRO_TOKEN:
		case OPEN_CALL_TOKEN:
		case CLOSE_CALL_TOKEN:
		case OPEN_PUSH_TOKEN:
		case CLOSE_PUSH_TOKEN:
		case INCLUDE_TOKEN:
		case SUBLABEL_TOKEN:
		case MACRO_ARG_SEP_TOKEN:
		case LABEL_TOKEN:
			t->type = c;
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		case STRING_TOKEN:
			t->type = c;
			t->text = pool_request(comp->code, 1);
			t->size = 0;
			while (comp->str.i < comp->str.size){
				c = comp->str.text[comp->str.i];
				comp->str.i += 1;
				if (c == '\n'){
					line += 1;
				}
				else if (c == '\\'){
					c = comp->str.text[comp->str.i];
					comp->str.i += 1;
					switch (c){
					case 'a': c = '\a'; break;
					case 'b': c = '\b'; break;
					case 'e': c = '\033'; break;
					case 'f': c = '\f'; break;
					case 'n': c = '\n'; break;
					case 'r': c = '\r'; break;
					case 't': c = '\t'; break;
					case 'v': c = '\v'; break;
					case '\\': c = '\\'; break;
					case '\'': c = '\''; break;
					case '"': c = '"'; break;
					case '?': c = '\?'; break;
					case '\n': line += 1; break;
					}
				}
				else if (c == STRING_TOKEN){
					t->text[t->size] = '\0';
					break;
				}
				t->text[t->size] = c;
				t->size += 1;
				pool_request(comp->code, 1);
			}
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		case '-':
			c = comp->str.text[comp->str.i];
			comp->str.i += 1;
			if (c != '0'){
				lex_symbol(comp, t);
				pool_request(comp->tok, sizeof(token));
				comp->token_count += 1;
				continue;
			}
			t->size += 1;
			negative = 1;
		case '0':
			c = comp->str.text[comp->str.i];
			ASSERT_LOCAL(c == 'x' || c == 'X', LEXERR " Malformed numeric prefix\n", line);
			comp->str.i += 1;
			t->size += 1;
			int64_t number = 0;
			byte index = 0;
			while (comp->str.i < comp->str.size && index < 16){
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
					if (number > 32767){
						t->type = DWORD_HEX_NUMERIC_TOKEN;
					}
					number *= -1;
				}
			}
			else if (index <= sizeof(uint32_t)*2){
				t->type = DWORD_HEX_NUMERIC_TOKEN;
				if (negative == 1){
					if (number > 0x7FFFFFFF){
						t->type = QWORD_HEX_NUMERIC_TOKEN;
					}
					number *= -1;
				}
			}
			else if (index <= sizeof(word)*2){
				t->type = QWORD_HEX_NUMERIC_TOKEN;
				if (negative == 1){
					ASSERT_LOCAL(number <= 0x7FFFFFFFFFFFFFFF, LEXERR " Too many bytes provided for negative numeric\n", line);
					number *= -1;
				}
			}
			else{
				ASSERT_LOCAL(0, LEXERR " Too many bytes provided for hex numeric\n", line);
			}
			t->data.number = number;
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		if (issymbol(c)){
			lex_symbol(comp, t);
			pool_request(comp->tok, sizeof(token));
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
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		OPCODE* o = OPCODE_map_access(&comp->opmap, (const char* const)t->text);
		if (o != NULL){
			t->type = OPCODE_TOKEN;
			t->data.opcode = *o;
			comp->str.text[comp->str.i] = copy;
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		REG_PARTITION* p = REG_PARTITION_map_access(&comp->partmap, (const char* const)t->text);
		if (p != NULL){
			t->type = PART_TOKEN;
			t->data.part = *p;
			comp->str.text[comp->str.i] = copy;
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		EXTERNAL_CALLS* ex = EXTERNAL_CALLS_map_access(&comp->extmap, (const char* const)t->text);
		if (ex != NULL){
			t->type = EXT_TOKEN;
			t->data.ext = *ex;
			comp->str.text[comp->str.i] = copy;
			pool_request(comp->tok, sizeof(token));
			comp->token_count += 1;
			continue;
		}
		comp->str.text[comp->str.i] = copy;
		pool_request(comp->tok, sizeof(token));
		comp->token_count += 1;
	}
	return 0;
}

byte
find_macros(compiler* const comp, pool* const aux){
	byte found = 0;
	word token_index = 0;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		token_index += 1;
		if (t.type != IDENTIFIER_TOKEN && t.type != SYMBOL_TOKEN){
			if (t.type == OPEN_MACRO_TOKEN){
				word start = token_index-1;
				t = comp->tokens[token_index];
				token_index += 1;
				ASSERT_LOCAL(t.type == IDENTIFIER_TOKEN || t.type == SYMBOL_TOKEN);
				word last_arg_index = token_index;
				int64_t nest = 0;
				word arg_index[8];
				word arg_length[8];
				word arg_count = 0;
				while (t.type != CLOSE_MACRO_TOKEN){
					t = comp->tokens[token_index];
					token_index += 1;
					if (t.type == MACRO_ARG_SEP_TOKEN){
						if (nest > 0){
							continue;
						}
						arg_index[arg_count] = last_arg_index;
						arg_length[arg_count] = (token_index - 1)  - last_arg_index;
						last_arg_index = token_index;
						arg_count += 1;
					}
					else if (t.type == CLOSE_MACRO_TOKEN){
						if (nest == 0){
							if (last_arg_index != token_index-1){
								arg_index[arg_count] = last_arg_index;
								arg_length[arg_count] = (token_index-1)-last_arg_index;
								arg_count += 1;
							}
							break;
						}
						nest -= 1;
					}
					else if (t.type == OPEN_MACRO_TOKEN){
						nest += 1;
					}
				}
				word end = token_index-1;
				if (comp->macro_calls == NULL){
					comp->macro_calls = pool_request(aux, sizeof(macro_call));
					comp->macro_calls->start = start;
					comp->macro_calls->end = end;
					comp->macro_calls->next = NULL;
					comp->macro_calls->prev = NULL;
					comp->macro_calls->arg_count = arg_count;
					for (byte i = 0;i<arg_count;++i){
						comp->macro_calls->arg_index[i] = arg_index[i];
						comp->macro_calls->arg_length[i] = arg_length[i];
					}
				}
				else{
					macro_call* node = comp->macro_calls;
					macro_call* new = pool_request(aux, sizeof(macro_call));
					new->start = start;
					new->end = end;
					new->next = NULL;
					new->prev = NULL;
					new->arg_count = arg_count;
					for (byte i = 0;i<arg_count;++i){
						new->arg_index[i] = arg_index[i];
						new->arg_length[i] = arg_length[i];
					}
					byte added = 0;
					while (node->next != NULL){
						if ((start > node->start) && (end < node->end)){
							added = 1;
							break;		
						}
						else if (start < node->start){
							if (node->prev == NULL){
								new->next = node;
								comp->macro_calls = new;
								added = 1;
								break;
							}
							new->prev = node->prev;
							node->prev = new;
							new->next = node;
							added = 1;
							break;
						}
						node = node->next;
					}
					if (node->next == NULL && (added == 0)){
						node->next = new;
						new->prev = node;
					}
				}
				found = 1;
			}
			continue;
		}
		word name_index = token_index-1;
		token mname = t;
		t = comp->tokens[token_index];
		token_index += 1;
		macro m = {
			.args = word_map_init(comp->mem),
			.start = 0,
			.end = 0
		};
		while (t.type == IDENTIFIER_TOKEN){
			char* argname = pool_request(aux, t.size+1);
			strncpy(argname, t.text, t.size);
			argname[t.size] = '\0';
			word* location = pool_request(aux, sizeof(word));
			*location = (token_index - 1) - name_index;
			byte dup = word_map_insert(&m.args, argname, location);
			ASSERT_LOCAL(dup == 0, PARSERR " Duplicate arg name for macro: '%s'" PARSERRFIX, argname, t.text);
			t = comp->tokens[token_index];
			token_index += 1;
		}
		if (t.type != EVAL_MACRO_TOKEN){
			continue;
		}
		t = comp->tokens[token_index];
		token_index += 1;
		ASSERT_LOCAL(t.type == OPEN_MACRO_TOKEN);
		int64_t nest = 0;
		m.start = token_index;
		while (t.type != CLOSE_MACRO_TOKEN){
			t = comp->tokens[token_index];
			token_index += 1;
			if (t.type == CLOSE_MACRO_TOKEN){
				if (nest == 0){
					m.end = token_index-1;
					break;
				}
				nest -= 1;
			}
			else if (t.type == OPEN_MACRO_TOKEN){
				nest += 1;
			}
		}
		m.defstart = name_index;
		char* macroname = pool_request(aux, mname.size+1);
		strncpy(macroname, mname.text, mname.size);
		macroname[mname.size] = '\0';
		macro* poolmacro = pool_request(comp->mem, sizeof(macro));
		*poolmacro = m;
		byte dup = macro_map_insert(&comp->macros, macroname, poolmacro);
		ASSERT_LOCAL(dup == 0, PARSERR " Duplicate macro '%s'" PARSERRFIX, macroname, mname.text);
	}
	return found;
}

byte
replace_macros(compiler* const comp, pool* const aux){
	token* new = pool_request(comp->tok_swp, sizeof(token));
	word token_index = 0;
	word new_index = 0;
	macro_call* node = comp->macro_calls;
	while (node != NULL){
		word elems = node->start - token_index;
		pool_request(comp->tok_swp, sizeof(token)*elems);
		memcpy(&new[new_index], &comp->tokens[token_index], elems*sizeof(token));
		new_index += elems;
		token_index += elems;
		token_index += 1;
		token macro_name = comp->tokens[token_index];
		char* name = macro_name.text;
		word size = macro_name.size;
		char save = name[size];
		name[size] = '\0';
		macro* target = macro_map_access(&comp->macros, name);
		ASSERT_LOCAL(target != NULL, " Macro '%s' which was marked as existing, does not\n", name);
		name[size] = save;
		token_index = target->start;
		while (token_index < target->end){
			token* t = &comp->tokens[token_index];
			token_index += 1;
			if (t->type == IDENTIFIER_TOKEN){
				char save = t->text[t->size];
				t->text[t->size] = '\0';
				word* isarg = word_map_access(&target->args, t->text);
				t->text[t->size] = save;
				if (isarg != NULL){
					word arg_index = (*isarg) - 1;
					ASSERT_LOCAL(arg_index <= node->arg_count, " Not enough arguments passed for macro %s\n", t->text);
					word start = node->arg_index[arg_index];
					word length = node->arg_length[arg_index];
					word end = start+length;
					while (start < end){
						new[new_index] = comp->tokens[start];
						start += 1;
						pool_request(comp->tok_swp, sizeof(token));
						new_index += 1;
					}
					continue;
				}
			}
			new[new_index] = *t;
			pool_request(comp->tok_swp, sizeof(token));
			new_index += 1;
		}
		token_index = node->end+1;
		node = node->next;
	}
	if (token_index < comp->token_count){
		word diff = comp->token_count - token_index;
		pool_request(comp->tok_swp, sizeof(token)*diff);
		memcpy(&new[new_index], &comp->tokens[token_index], diff*sizeof(token));
		new_index += diff;
	}
	comp->token_count = new_index;
	comp->tokens = new;
	return 0;
}

word
parse_register(compiler* const comp, word token_index, byte* r){
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

void 
parse_string_bytes(compiler* const comp, word token_index, data_tree* data){
	data->type = BYTE_DATA;
	token t = comp->tokens[token_index];
	word remainder_offset = 8-(t.size % 8);
	data->data.bytes.size = t.size+remainder_offset;
	data->data.bytes.raw = pool_request(comp->mem, t.size+remainder_offset);
	for (word i = 0;i<remainder_offset;++i){
		data->data.bytes.raw[i] = 0;
	}
	for (word i = 0;i<t.size;++i){
		data->data.bytes.raw[remainder_offset+i] = t.text[t.size-(1+i)];
	}
}

word
parse_call_block(compiler* const comp, bsms* const sublabels, word token_index, call_tree* data){
	call_tree* last = data;
	token t = comp->tokens[token_index];
	while (token_index < comp->token_count){
		t = comp->tokens[token_index];
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
		case STRING_TOKEN:
			data->type = PUSH_ARG;
			data->data.push = pool_request(comp->mem, sizeof(data_tree));
			data->data.push->next = NULL;
			parse_string_bytes(comp, token_index-1, data->data.push);
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
			ASSERT_ERR(0);
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
	ASSERT_LOCAL(0, PARSERR " Unexpected end to procedure call" PARSERRFIX, t.text);
	return 0;
}

word
parse_byte_sequence(compiler* const comp, word token_index, data_tree* data){
	data->data.bytes.size = 0;
	word initial_index = token_index;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		if (t.type == QWORD_HEX_NUMERIC_TOKEN){
			data->data.bytes.size += 8;
		}
		else if (t.type == DWORD_HEX_NUMERIC_TOKEN){
			data->data.bytes.size += 4;
		}
		else if (t.type == SHORT_HEX_NUMERIC_TOKEN){
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
		case QWORD_HEX_NUMERIC_TOKEN:
			data->data.bytes.raw[index] = (t.data.number & 0xFF00000000000000) >> 0x38;
			index += 1;
			data->data.bytes.raw[index] = (t.data.number & 0xFF000000000000) >> 0x30;
			index += 1;
			data->data.bytes.raw[index] = (t.data.number & 0xFF0000000000) >> 0x28;
			index += 1;
			data->data.bytes.raw[index] = (t.data.number & 0xFF00000000) >> 0x20;
			index += 1;
		case DWORD_HEX_NUMERIC_TOKEN:
			data->data.bytes.raw[index] = (t.data.number & 0xFF000000) >> 0x18;
			index += 1;
			data->data.bytes.raw[index] = (t.data.number & 0xFF0000) >> 0x10;
			index += 1;
		case SHORT_HEX_NUMERIC_TOKEN:
			data->data.bytes.raw[index] = (t.data.number & 0xFF00) >> 0x8;
			index += 1;
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

word
parse_push_block(compiler* const comp, bsms* const sublabels, word token_index, data_tree* data){
	data_tree* last = data;
	while (token_index < comp->token_count){
		token t = comp->tokens[token_index];
		token_index += 1;
		switch (t.type){
		case QWORD_HEX_NUMERIC_TOKEN:
		case DWORD_HEX_NUMERIC_TOKEN:
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
		case STRING_TOKEN:
			parse_string_bytes(comp, token_index-1, data);
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
			data->data.code->prev = NULL;
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

word
parse_3reg(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
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

word
parse_2reg(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
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

word
parse_2reg_byte(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
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

word
parse_reg_short(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code){
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

word
parse_instruction_block(compiler* const comp, bsms* const sublabels, word token_index, code_tree* code){
	token t = comp->tokens[token_index];
	ASSERT_LOCAL(t.type == OPCODE_TOKEN, PARSERR " Expected opcode in parse instruction start" PARSERRFIX, t.text);
	code->type = INSTRUCTION_BLOCK;
	code->code.instructions = pool_request(comp->code, 4*BLOCK_START_SIZE);
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
				code->code.instructions = pool_request(comp->code, 4*5);
				code->code.instructions[0] = opc;
				code->code.instructions[1] = 0;
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
			case OPCODE_TOKEN:
				code->code.instructions[instruction_index] = opc;
				code->code.instructions[instruction_index+1] = 2;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = REG(FULL, AR);
				token_index -= 1;
				break;
			default:
				code->code.instructions[instruction_index] = opc;
				code->code.instructions[instruction_index+1] = 1;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = 0;
				token_index -= 1;
				break;
			}
		}
		else{
			switch (t.data.opcode){
			case LDA: case STA: case ADD: case SUB:
			case MUL: case DIV: case MOD: case AND:
			case OR: case SHL: case SHR: case XOR:
				token_index = parse_3reg(comp, t.data.opcode, instruction_index, token_index, code);
				ASSERT_ERR(0);
				break;
			case LDS: case STS: case ADS: case SUS:
			case MUS: case DIS: case MOS: case ANS:
			case ORS: case SLS: case SRS: case XRS:
			case CMS:
				token_index = parse_reg_short(comp, t.data.opcode, instruction_index, token_index, code);
				ASSERT_ERR(0);
				break;
			case MOV: case SWP: case ADI: case SUI:
			case MUI: case DII: case MOI: case ANI:
			case ORI: case SLI: case SRI: case XRI:
			case INV: case COM: case CMP:
				token_index = parse_2reg(comp, t.data.opcode, instruction_index, token_index, code);
				ASSERT_ERR(0);
				break;
			case LDI: case STB:
				token_index = parse_2reg_byte(comp, t.data.opcode, instruction_index, token_index, code);
				ASSERT_ERR(0);
				break;
			case INI: case COI: case RET: case PSH:
			case POP: case EXR:
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
			case EXT:
				token b = comp->tokens[token_index];
				ASSERT_LOCAL(b.type == EXT_TOKEN, PARSERR " Expected external directive" PARSERRFIX, b.text);
				token_index += 1;
				code->code.instructions[instruction_index] = t.data.opcode;
				code->code.instructions[instruction_index+1] = b.data.ext;
				code->code.instructions[instruction_index+2] = 0;
				code->code.instructions[instruction_index+3] = 0;
				break;
			case NOP: case REI: case CAL:
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
			pool_request(comp->code, 4*code->code.instruction_count);
			instruction_capacity *= 2;
		}
	}
	return token_index;
}

word
parse_code(compiler* const comp, bsms* const sublabels, word token_index, code_tree* ir, TOKEN terminator){
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
			ir->prev = last;
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
			ir->prev = last;
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
			ir->prev = last;
			ir->next = NULL;
			continue;
		case STRING_TOKEN:
			ir->type = PUSH_BLOCK;
			ir->data.push = pool_request(comp->mem, sizeof(data_tree));
			ir->data.push->next = NULL;
			parse_string_bytes(comp, token_index-1, ir->data.push);
			ir->next = pool_request(comp->mem, sizeof(code_tree));
			last = ir;
			ir = ir->next;
			ir->labeling = NOT_LABELED;
			ir->prev = last;
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
			ASSERT_LOCAL(t.type == LABEL_TOKEN, PARSERR " Expected label" PARSERRFIX, t.text);
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
			if (t.type == IDENTIFIER_TOKEN){
				int64_t nest = 0;
				while (t.type != CLOSE_MACRO_TOKEN){
					t = comp->tokens[token_index];
					token_index += 1;
					if (t.type == CLOSE_MACRO_TOKEN){
						if (nest == 0){
							break;
						}
						nest -= 1;
					}
					else if (t.type == OPEN_MACRO_TOKEN){
						nest += 1;
					}
				}
				continue;
			}
			ASSERT_LOCAL(t.type == LABEL_TOKEN, PARSERR " Expected label" PARSERRFIX, t.text);
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

byte
block_scope_add_member(block_scope_map* const block, token t, code_tree* member){
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
	char* new_name = pool_request(block->mem, size+1);
	strncpy(new_name, name, size);
	new_name[size] = '\0';
	block_scope_map_insert(block, new_name, node);
	return 0;
}

code_tree*
block_scope_check_member(block_scope_map* const block, token t, code_tree** ref){
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
	char* new_name = pool_request(block->mem, size+1);
	strncpy(new_name, name, size);
	new_name[size] = '\0';
	block_scope_map_insert(block, new_name, node);
	return NULL;
}

void
bsms_push(bsms* stack){
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

void
bsms_pop(bsms* stack){
	stack->size -= 1;
}

byte
check_label_bucket(compiler* const comp, block_scope_map_bucket* bucket){
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

byte
remaining_labels(compiler* const comp, block_scope_map* const block){
	HASHMAP_ITERATE(i){
		block_scope_map_bucket* bucket = &block->buckets[i];
		check_label_bucket(comp, bucket);
		ASSERT_ERR(0);
	}
	return 0;
}

byte
parse_tokens(compiler* const comp){
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
	comp->ir->prev = NULL;
	parse_code(comp, &sublabels, token_index, comp->ir, NONE_TOKEN);
	remaining_labels(comp, &sublabels.map[sublabels.size-1]);
	pool_dealloc(&sublabel_pool);
	ASSERT_ERR(0);
	remaining_labels(comp, &comp->labels.map[comp->labels.size-1]);
	ASSERT_ERR(0);
	return 0;
}

#define INDENT_SHOW(d)\
	for (byte di = 0;di<d;++di){\
		printf(" ");\
	}

byte
show_call(compiler* const comp, call_tree* call, word depth){
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

byte
show_data(compiler* const comp, data_tree* data, word depth){
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

byte
show_block(compiler* const comp, code_tree* code, word depth){
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
			if (code->dest.text == NULL){
				break;
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
			if (code->dest.text == NULL){
				break;
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
			if (code->code.instructions == NULL){
				break;
			}
			INDENT_SHOW(depth) printf("PRE-GEN INSTRUCTIONS:\n");
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
		case PUSH_BLOCK:
			INDENT_SHOW(depth) printf("PUSH:\n");
			show_data(comp, code->data.push, depth+1);
			ASSERT_ERR(0);
			if (code->code.instructions == NULL){
				break;
			}
			INDENT_SHOW(depth) printf("PRE-GEN INSTRUCTIONS:\n");
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
		}
		code = code->next;
	}
	return 0;
}

void
show_tokens(compiler* const comp){
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
		case EXT_TOKEN:
			printf("EXTERNAL %u ", comp->tokens[i].data.ext);
			break;
		case OPEN_MACRO_TOKEN:
			printf("OPEN MACRO ");
			break;
		case CLOSE_MACRO_TOKEN:
			printf("CLOSE MACRO ");
			break;
		case EVAL_MACRO_TOKEN:
			printf("MACRO EVAL ");
			break;
		case MACRO_ARG_SEP_TOKEN:
			printf("ARG SEP ");
			break;
		case SYMBOL_TOKEN:
			printf("SYMBOL TOKEN: ");
			break;
		case OPEN_CALL_TOKEN:
			printf("OPEN CALL ");
			break;
		case CLOSE_CALL_TOKEN:
			printf("CLOSE CALL ");
			break;
		case OPEN_PUSH_TOKEN:
			printf("OPEN PUSH ");
			break;
		case CLOSE_PUSH_TOKEN:
			printf("CLOSE PUSH ");
			break;
		case STRING_TOKEN:
			printf("STRING ");
			break;
		case SUBLABEL_TOKEN:
			printf("SUBLABEL ");
			break;
		case LABEL_TOKEN:
			printf("LABEL ");
			break;
		case INCLUDE_TOKEN:
			printf("INCLUSION ");
			break;
		case SHORT_HEX_NUMERIC_TOKEN:
			printf("SHORT ");
			break;
		case BYTE_HEX_NUMERIC_TOKEN:
			printf("BYTE ");
			break;
		case DWORD_HEX_NUMERIC_TOKEN:
			printf("D WORD ");
			break;
		case QWORD_HEX_NUMERIC_TOKEN:
			printf("Q WORD ");
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
		if ((i+1) % 4 == 0){
			printf("\n");
		}
	}
	printf("\n");
}

code_tree*
pregen_push(compiler* const comp, ltms* const sublines, code_tree* basic_block, data_tree* push){
	word instruction_index = basic_block->code.instruction_count*4;
	while (push != NULL){
		switch (push->type){
		case BYTE_DATA:
			word under_round = 0;
			word remainder = push->data.bytes.size;
			if (remainder > 7){
				remainder %= 8;
				under_round = push->data.bytes.size-remainder;
			}
			pool_request(comp->code, (4*5)+(under_round*4*5));
			for (word i = 0;i<under_round;){
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(L16, AR);
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(LM16, AR);
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(RM16, AR);
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(R16, AR);
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = push->data.bytes.raw[i++];
				basic_block->code.instructions[instruction_index++] = PSH;
				basic_block->code.instructions[instruction_index++] = REG(FULL, AR);
				basic_block->code.instructions[instruction_index++] = 0;
				basic_block->code.instructions[instruction_index++] = 0;
				basic_block->code.instruction_count += 5;
				comp->lines.line[comp->lines.size-1] += 5;
				sublines->line[sublines->size-1] += 5;
			}
			if (remainder != 0){
				byte line[8] = {0};
				for (word i = 0;i<remainder;++i){
					line[i] = push->data.bytes.raw[under_round+i];
				}
				word index = 0;
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(L16, AR);
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(LM16, AR);
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(RM16, AR);
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = LDS;
				basic_block->code.instructions[instruction_index++] = REG(R16, AR);
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = line[index++];
				basic_block->code.instructions[instruction_index++] = PSH;
				basic_block->code.instructions[instruction_index++] = REG(FULL, AR);
				basic_block->code.instructions[instruction_index++] = 0;
				basic_block->code.instructions[instruction_index++] = 0;
				basic_block->code.instruction_count += 5;
				comp->lines.line[comp->lines.size-1] += 5;
				sublines->line[sublines->size-1] += 5;
			}
			break;
		case NEST_DATA:
			basic_block = pregen_push(comp, sublines, basic_block, push->data.nest);
			instruction_index = basic_block->code.instruction_count*4;
			break;
		case CODE_DATA:
			ltms_push(sublines);
			ltms_push(&comp->lines);
			pregenerate(comp, sublines, push->data.code);
			while (comp->lines.changed[comp->lines.size-1] || sublines->changed[sublines->size-1]){
				loc_thunk_map_empty(&sublines->map[sublines->size-1]);
				loc_thunk_map_empty(&comp->lines.map[comp->lines.size-1]);
				sublines->line[sublines->size-1] = 0;
				sublines->changed[sublines->size-1] = 0;
				comp->lines.line[comp->lines.size-1] = 0;
				comp->lines.changed[comp->lines.size-1] = 0;
				correct_offsets(comp, sublines, push->data.code);
			}
			ltms_pop(&comp->lines);
			ltms_pop(sublines);
			byte* old = basic_block->code.instructions;
			basic_block->code.instructions = pool_request(comp->code, basic_block->code.instruction_count*4);
			for (word i = 0;i<basic_block->code.instruction_count*4;++i){
				basic_block->code.instructions[i] = old[i];
			}
			code_tree* code = push->data.code;
			while (code->next != NULL){
				code = code->next;
			}
			while (code != NULL){
				word n = code->code.instruction_count * 4;
				byte hi = 0;
				for (word i = n;i>0;){
					if (hi == 1){
						pool_request(comp->code, 4*3);
						basic_block->code.instructions[instruction_index++] = LDS;
						basic_block->code.instructions[instruction_index++] = REG(RM16, AR);
						basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
						basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
						basic_block->code.instructions[instruction_index++] = LDS;
						basic_block->code.instructions[instruction_index++] = REG(R16, AR);
						basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
						basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
						basic_block->code.instructions[instruction_index++] = PSH;
						basic_block->code.instructions[instruction_index++] = REG(FULL, AR);
						basic_block->code.instructions[instruction_index++] = 0;
						basic_block->code.instructions[instruction_index++] = 0;
						basic_block->code.instruction_count += 3;
						comp->lines.line[comp->lines.size-1] += 3;
						sublines->line[sublines->size-1] += 3;
						hi = 0;
						continue;
					}
					pool_request(comp->code, 4*2);
					basic_block->code.instructions[instruction_index++] = LDS;
					basic_block->code.instructions[instruction_index++] = REG(L16, AR);
					basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
					basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
					basic_block->code.instructions[instruction_index++] = LDS;
					basic_block->code.instructions[instruction_index++] = REG(LM16, AR);
					basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
					basic_block->code.instructions[instruction_index++] = code->code.instructions[--i];
					basic_block->code.instruction_count += 2;
					comp->lines.line[comp->lines.size-1] += 2;
					sublines->line[sublines->size-1] += 2;
					hi = 1;
				}
				if (hi == 1){
					pool_request(comp->code, 4*3);
					basic_block->code.instructions[instruction_index++] = LDS;
					basic_block->code.instructions[instruction_index++] = REG(RM16, AR);
					basic_block->code.instructions[instruction_index++] = 0;
					basic_block->code.instructions[instruction_index++] = 0;
					basic_block->code.instructions[instruction_index++] = LDS;
					basic_block->code.instructions[instruction_index++] = REG(R16, AR);
					basic_block->code.instructions[instruction_index++] = 0;
					basic_block->code.instructions[instruction_index++] = 0;
					basic_block->code.instructions[instruction_index++] = PSH;
					basic_block->code.instructions[instruction_index++] = REG(FULL, AR);
					basic_block->code.instructions[instruction_index++] = 0;
					basic_block->code.instructions[instruction_index++] = 0;
					basic_block->code.instruction_count += 3;
					comp->lines.line[comp->lines.size-1] += 3;
					sublines->line[sublines->size-1] += 3;
				}
				code = code->prev;
			}
			break;
		}
		push = push->next;
	}
	return basic_block;
}

code_tree*
pregen_call(compiler* const comp, ltms* const sublines, code_tree* basic_block, call_tree* call){
	word instruction_index = basic_block->code.instruction_count*4;
	basic_block->code.instructions[instruction_index++] = CAL;
	basic_block->code.instructions[instruction_index++] = 0;
	basic_block->code.instructions[instruction_index++] = 0;
	basic_block->code.instructions[instruction_index++] = 0;
	basic_block->code.instruction_count += 1;
	comp->lines.line[comp->lines.size-1] += 1;
	sublines->line[sublines->size-1] += 1;
	call_tree* function = call;
	call = call->next;
	if (function->type == CALL_ARG){
		basic_block = pregen_call(comp, sublines, basic_block, function->data.call);
		instruction_index = basic_block->code.instruction_count * 4;
		pool_request(comp->code, 4);
		basic_block->code.instructions[instruction_index++] = POP;
		basic_block->code.instructions[instruction_index++] = REG(FULL, LR);
		basic_block->code.instructions[instruction_index++] = 0;
		basic_block->code.instructions[instruction_index++] = 0;
		basic_block->code.instruction_count += 1;
		comp->lines.line[comp->lines.size-1] += 1;
		sublines->line[sublines->size-1] += 1;
		function = NULL;
	}
	else if (function->type == PUSH_ARG){
		basic_block = pregen_push(comp, sublines, basic_block, call->data.push);
		instruction_index = basic_block->code.instruction_count * 4;
		pool_request(comp->code, 4);
		basic_block->code.instructions[instruction_index++] = POP;
		basic_block->code.instructions[instruction_index++] = REG(FULL, LR);
		basic_block->code.instructions[instruction_index++] = 0;
		basic_block->code.instructions[instruction_index++] = 0;
		basic_block->code.instruction_count += 1;
		comp->lines.line[comp->lines.size-1] += 1;
		sublines->line[sublines->size-1] += 1;
		function = NULL;
	}
	while (call != NULL){
		switch (call->type){
		case CALL_ARG:
			pool_request(comp->code, 4);
			basic_block = pregen_call(comp, sublines, basic_block, call->data.call);
			instruction_index = basic_block->code.instruction_count * 4;
			break;
		case PUSH_ARG:
			basic_block = pregen_push(comp, sublines, basic_block, call->data.push);
			instruction_index = basic_block->code.instruction_count * 4;
			break;
		case REG_ARG:
			pool_request(comp->code, 4);
			basic_block->code.instructions[instruction_index++] = PSH;
			basic_block->code.instructions[instruction_index++] = call->data.reg;
			basic_block->code.instructions[instruction_index++] = 0;
			basic_block->code.instructions[instruction_index++] = 0;
			basic_block->code.instruction_count += 1;
			comp->lines.line[comp->lines.size-1] += 1;
			sublines->line[sublines->size-1] += 1;
			break;
		case LABEL_ARG:
		case SUBLABEL_ARG:
			pool_request(comp->code, 4*5);
			basic_block->code.instructions[instruction_index++] = LDS;
			basic_block->code.instructions[instruction_index++] = REG(L16, AR);
			basic_block->code.instructions[instruction_index++] = 0xDE;
			basic_block->code.instructions[instruction_index++] = 0x57;
			basic_block->code.instructions[instruction_index++] = LDS;
			basic_block->code.instructions[instruction_index++] = REG(LM16, AR);
			basic_block->code.instructions[instruction_index++] = 0xAD;
			basic_block->code.instructions[instruction_index++] = 0xD0;
			basic_block->code.instructions[instruction_index++] = LDS;
			basic_block->code.instructions[instruction_index++] = REG(RM16, AR);
			basic_block->code.instructions[instruction_index++] = 0xDE;
			basic_block->code.instructions[instruction_index++] = 0x57;
			basic_block->code.instructions[instruction_index++] = LDS;
			basic_block->code.instructions[instruction_index++] = REG(R16, AR);
			basic_block->code.instructions[instruction_index++] = 0xAD;
			basic_block->code.instructions[instruction_index++] = 0xD1;
			basic_block->code.instructions[instruction_index++] = PSH;
			basic_block->code.instructions[instruction_index++] = REG(FULL, AR);
			basic_block->code.instructions[instruction_index++] = 0;
			basic_block->code.instructions[instruction_index++] = 0;
			basic_block->code.instruction_count += 5;
			comp->lines.line[comp->lines.size-1] += 5;
			sublines->line[sublines->size-1] += 5;
			if (call->type == LABEL_ARG){
				loc_thunk_check_member(&comp->lines, call->data.labeling.label, replace_call_arg, call->data.labeling.dest_block);
			}
			else if (call->type == SUBLABEL_ARG){
				loc_thunk_check_member(sublines, call->data.labeling.label, replace_call_arg, call->data.labeling.dest_block);
			}
			break;
		case NUMERIC_ARG:
			pool_request(comp->code, 4);
			basic_block->code.instructions[instruction_index++] = PSS;
			basic_block->code.instructions[instruction_index++] = (call->data.number & 0xFF00) >> 8;
			basic_block->code.instructions[instruction_index++] = (call->data.number & 0xFF);
			basic_block->code.instructions[instruction_index++] = 0;
			comp->lines.line[comp->lines.size-1] += 1;
			sublines->line[sublines->size-1] += 1;
			basic_block->code.instruction_count += 1;
			break;
		}
		call = call->next;
	}
	code_tree* new_block = pool_request(comp->mem, sizeof(code_tree));
	new_block->type = INSTRUCTION_JUMP;
	new_block->labeling = NOT_LABELED;
	new_block->next = basic_block->next;
	if (basic_block->next != NULL){
		new_block->next->prev = new_block;
	}
	basic_block->next = new_block;
	new_block->prev = basic_block;
	new_block->code.instructions = pool_request(comp->code, 4*5);
	new_block->code.instruction_count = 1;
	instruction_index = 0;
	if (function == NULL){
		new_block->code.instructions[instruction_index++] = BNC;
		new_block->code.instructions[instruction_index++] = 2;
		new_block->code.instructions[instruction_index++] = 0;
		new_block->code.instructions[instruction_index++] = REG(FULL, LR);
		comp->lines.line[comp->lines.size-1] += 1;
		sublines->line[sublines->size-1] += 1;
	}
	else{
		switch (function->type){
		case REG_ARG:
			new_block->code.instructions[instruction_index++] = BNC;
			new_block->code.instructions[instruction_index++] = 2;
			new_block->code.instructions[instruction_index++] = 0;
			new_block->code.instructions[instruction_index++] = function->data.reg;
			comp->lines.line[comp->lines.size-1] += 1;
			sublines->line[sublines->size-1] += 1;
			break;
		case LABEL_ARG:
			new_block->dest = function->data.labeling.label;
			new_block->dest_block = function->data.labeling.dest_block;
			new_block->code.instructions[instruction_index++] = BNC;
			new_block->code.instructions[instruction_index++] = 2;
			new_block->code.instructions[instruction_index++] = 0;
			new_block->code.instructions[instruction_index++] = 0;
			loc_thunk_check_member(&comp->lines, new_block->dest, replace_call_dest, new_block);
			comp->lines.line[comp->lines.size-1] += 1;
			sublines->line[sublines->size-1] += 1;
			break;
		case SUBLABEL_ARG:
			new_block->type = INSTRUCTION_SUBJUMP;
			new_block->dest = function->data.labeling.label;
			new_block->dest_block = function->data.labeling.dest_block;
			new_block->code.instructions[instruction_index++] = BNC;
			new_block->code.instructions[instruction_index++] = 2;
			new_block->code.instructions[instruction_index++] = 0;
			new_block->code.instructions[instruction_index++] = 0;
			loc_thunk_check_member(sublines, new_block->dest, replace_call_dest, new_block);
			comp->lines.line[comp->lines.size-1] += 1;
			sublines->line[sublines->size-1] += 1;
			break;
		case NUMERIC_ARG:
			new_block->code.instructions[instruction_index++] = BNC;
			new_block->code.instructions[instruction_index++] = 1;
			new_block->code.instructions[instruction_index++] = (function->data.number & 0xFF00) >> 8;
			new_block->code.instructions[instruction_index++] = (function->data.number & 0xFF);
			comp->lines.line[comp->lines.size-1] += 1;
			sublines->line[sublines->size-1] += 1;
			break;
		case CALL_ARG:
		case PUSH_ARG:
		}
	}
	code_tree* next_block = pool_request(comp->mem, sizeof(code_tree));
	next_block->type = INSTRUCTION_BLOCK;
	next_block->labeling = NOT_LABELED;
	next_block->next = new_block->next;
	if (new_block->next != NULL){
		next_block->next->prev = next_block;
	}
	next_block->prev = new_block;
	new_block->next = next_block;
	next_block->code.instructions = pool_request(comp->code, 4);
	next_block->code.instruction_count = 0;
	return next_block;
}

byte
replace_call_arg(code_tree* jump, word jumpline, word line){
	word index = 2;
	jump->code.instructions[index++] = (line & 0xFF00000000000000) >> 0x38;
	jump->code.instructions[index++] = (line & 0xFF000000000000) >> 0x30;
	index += 2;
	jump->code.instructions[index++] = (line & 0xFF0000000000) >> 0x28;
	jump->code.instructions[index++] = (line & 0xFF00000000) >> 20;
	index += 2;
	jump->code.instructions[index++] = (line & 0xFF000000) >> 0x18;
	jump->code.instructions[index++] = (line & 0xFF0000) >> 0x10;
	index += 2;
	jump->code.instructions[index++] = (line & 0xFF00) >> 0x8;
	jump->code.instructions[index++] = (line & 0xFF);
	return 0;
}

byte
replace_call_dest(code_tree* jump, word jumpline, word line){
	word offset = 0;
	byte neg = 0;
	if (jumpline > line){
		offset = jumpline - line;
		neg = 1;
	}
	else {
		offset = line - jumpline;
	}
	word index = 0;
	byte opc = jump->code.instructions[0];
	byte changed = 0;
	if (offset <= 0x7FFF){
		if (jump->code.instruction_count > 1){
			opc = jump->code.instructions[4*4];
			jump->code.instruction_count = 1;
			changed = 1;
		}
		jump->code.instructions[index] = opc;
		jump->code.instructions[index+1] = 1;
		if (neg == 1){
			int16_t temp = offset;
			temp *= -1;
			jump->code.instructions[index+2] = (temp & 0xFF00) >> 8;
			jump->code.instructions[index+3] = (temp & 0xFF);
		}
		else{
			jump->code.instructions[index+2] = (offset & 0xFF00) >> 8;
			jump->code.instructions[index+3] = (offset & 0xFF);
		}
		return changed;
	}
	if (jump->code.instruction_count == 1){
		jump->code.instruction_count = 5;
		changed = 1;
	}
	else{
		opc = jump->code.instructions[4*4];
	}
	if (offset <= 0x7FFFFFFFFFFFFFFF){
		if (neg == 1){
			int64_t temp = offset;
			temp *= -1;
			offset = *(word*)(&temp);
		}
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(L16, LR);
		jump->code.instructions[index++] = (offset & 0xFF00000000000000) >> 0x38;
		jump->code.instructions[index++] = (offset & 0xFF000000000000) >> 0x30;
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(LM16, LR);
		jump->code.instructions[index++] = (offset & 0xFF0000000000) >> 0x28;
		jump->code.instructions[index++] = (offset & 0xFF00000000) >> 0x20;
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(RM16, LR);
		jump->code.instructions[index++] = (offset & 0xFF000000) >> 0x18;
		jump->code.instructions[index++] = (offset & 0xFF0000) >> 0x10;
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(R16, LR);
		jump->code.instructions[index++] = (offset & 0xFF00) >> 8;
		jump->code.instructions[index++] = (offset & 0xFF);
		jump->code.instructions[index] = opc;
		jump->code.instructions[index+1] = 0;
	}
	else{
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(L16, LR);
		jump->code.instructions[index++] = (line & 0xFF00000000000000) >> 0x38;
		jump->code.instructions[index++] = (line & 0xFF000000000000) >> 0x30;
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(LM16, LR);
		jump->code.instructions[index++] = (line & 0xFF0000000000) >> 0x28;
		jump->code.instructions[index++] = (line & 0xFF00000000) >> 0x20;
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(RM16, LR);
		jump->code.instructions[index++] = (line & 0xFF000000) >> 0x18;
		jump->code.instructions[index++] = (line & 0xFF0000) >> 0x10;
		jump->code.instructions[index++] = LDS;
		jump->code.instructions[index++] = REG(R16, LR);
		jump->code.instructions[index++] = (line & 0xFF00) >> 8;
		jump->code.instructions[index++] = (line & 0xFF);
		jump->code.instructions[index] = opc;
		jump->code.instructions[index+1] = 2;
	}
	jump->code.instructions[index+2] = 0;
	jump->code.instructions[index+3] = REG(FULL, LR);
	return changed;
}

void
pregenerate(compiler* const comp, ltms* const sublines, code_tree* basic_block){
	while (basic_block != NULL){
		if (basic_block->labeling == LABELED){
			comp->lines.changed[comp->lines.size-1] |= sublines->changed[sublines->size-1];
			loc_thunk_map* subthunk = &sublines->map[sublines->size-1];
			if (sublines->size == 1){
				pool_empty(subthunk->mem);
			}
			loc_thunk_map_empty(subthunk);
			loc_thunk_add_member(&comp->lines, basic_block->label);
		}
		else if (basic_block->labeling == SUBLABELED){
			loc_thunk_add_member(sublines, basic_block->label);
		}
		if (basic_block->type == PUSH_BLOCK){
			basic_block->code.instructions = pool_request(comp->code, 4);
			basic_block->code.instruction_count = 0;
			basic_block = pregen_push(comp, sublines, basic_block, basic_block->data.push);
		}
		else if (basic_block->type == CALL_BLOCK){
			basic_block->code.instructions = pool_request(comp->code, 4);
			basic_block->code.instruction_count = 0;
			basic_block = pregen_call(comp, sublines, basic_block, basic_block->data.call);
		}
		else {
			if (basic_block->type == INSTRUCTION_JUMP && basic_block->dest.text != NULL){
				loc_thunk_check_member(&comp->lines, basic_block->dest, replace_call_dest, basic_block);
			}
			else if (basic_block->type == INSTRUCTION_SUBJUMP && basic_block->dest.text != NULL){
				loc_thunk_check_member(sublines, basic_block->dest, replace_call_dest, basic_block);
			}
			comp->lines.line[comp->lines.size-1] += basic_block->code.instruction_count;
			sublines->line[sublines->size-1] += basic_block->code.instruction_count;
		}
		basic_block = basic_block->next;
	}
}

void
correct_offsets(compiler* const comp, ltms* const sublines, code_tree* basic_block){
	while (basic_block != NULL){
		if (basic_block->labeling == LABELED){
			comp->lines.changed[comp->lines.size-1] = sublines->changed[sublines->size-1];
			loc_thunk_map* subthunk = &sublines->map[sublines->size-1];
			if (sublines->size == 1){
				pool_empty(subthunk->mem);
			}
			loc_thunk_map_empty(subthunk);
			loc_thunk_add_member(&comp->lines, basic_block->label);
		}
		else if (basic_block->labeling == SUBLABELED){
			loc_thunk_add_member(sublines, basic_block->label);
		}
		if (basic_block->type == INSTRUCTION_JUMP && basic_block->dest.text != NULL){
			loc_thunk_check_member(&comp->lines, basic_block->dest, replace_call_dest, basic_block);
		}
		else if (basic_block->type == INSTRUCTION_SUBJUMP && basic_block->dest.text != NULL){
			loc_thunk_check_member(sublines, basic_block->dest, replace_call_dest, basic_block);
		}
		comp->lines.line[comp->lines.size-1] += basic_block->code.instruction_count;
		sublines->line[sublines->size-1] += basic_block->code.instruction_count;
		basic_block = basic_block->next;
	}
}

byte
loc_thunk_add_member(ltms* const stack, token t){
	loc_thunk_map* thunk = &stack->map[stack->size-1];
	char* name = t.text;
	word size = t.size;
	char save = name[size];
	name[size] = '\0';
	loc_thunk* node = loc_thunk_map_access(thunk, name);
	name[size] = save;
	if (node != NULL){
		if (node->type == FULFILLED_MEMBER){
			return 1;
		}
		node->type = FULFILLED_MEMBER;
		while (node != NULL){
			stack->changed[stack->size-1] |= node->f(node->jump, node->jump_line, stack->line[stack->size-1]);
			node = node->next;
		}
		return 0;
	}
	node = pool_request(thunk->mem, sizeof(loc_thunk));
	node->type = FULFILLED_MEMBER;
	node->next = NULL;
	node->label = t;
	node->line = stack->line[stack->size-1];
	node->jump = NULL;
	char* new_name = pool_request(thunk->mem, size+1);
	strncpy(new_name, name, size);
	new_name[size] = '\0';
	loc_thunk_map_insert(thunk, new_name, node);
	return 0;
}

void
loc_thunk_check_member(ltms* const stack, token t, byte(*f)(code_tree*, word, word), code_tree* member){
	loc_thunk_map* thunk = &stack->map[stack->size-1];
	char* name = t.text;
	word size = t.size;
	char save = name[size];
	name[size] = '\0';
	loc_thunk* node = loc_thunk_map_access(thunk, name);
	name[size] = save;
	if (node != NULL){
		if (node->type == FULFILLED_MEMBER){
			stack->changed[stack->size-1] |= f(member, stack->line[stack->size-1], node->line);
			return;
		}
		loc_thunk* temp = node->next;
		node->next = pool_request(thunk->mem, sizeof(loc_thunk));
		node = node->next;
		node->next = temp;
		node->type = PENDING_MEMBER;
		node->jump = member;
		node->label = t;
		node->line = 0;
		node->jump_line = stack->line[stack->size-1];
		node->f = f;
		return;
	}
	node = pool_request(thunk->mem, sizeof(loc_thunk));
	node->type = PENDING_MEMBER;
	node->jump = member;
	node->next = NULL;
	node->label = t;
	node->line = 0;
	node->jump_line = stack->line[stack->size-1];
	node->f = f;
	char* new_name = pool_request(thunk->mem, size+1);
	strncpy(new_name, name, size);
	new_name[size] = '\0';
	loc_thunk_map_insert(thunk, new_name, node);
	return;
}

void
ltms_push(ltms* stack){
	if (stack->size >= stack->capacity){
		byte old_cap = stack->capacity;
		stack->capacity *= 2;
		loc_thunk_map* old = stack->map;
		stack->map = pool_request(old->mem, sizeof(loc_thunk_map)*stack->capacity);
		for (byte i = 0;i<old_cap;++i){
			stack->map[i] = old[i];
		}
	}
	stack->map[stack->size] = loc_thunk_map_init(stack->map[0].mem);
	stack->line[stack->size] = 0;
	stack->changed[stack->size] = 0;
	stack->size += 1;
}

void
ltms_pop(ltms* stack){
	stack->size -= 1;
	stack->changed[stack->size-1] |= stack->changed[stack->size];
}

byte
backpass(compiler* const comp){
	{
		comp->lines.map = pool_request(comp->mem, sizeof(loc_thunk_map)*PUSH_LABEL_SCOPE_LIMIT);
		comp->lines.line = pool_request(comp->mem, sizeof(loc_thunk_map)*PUSH_LABEL_SCOPE_LIMIT);
		comp->lines.size = 1;
		comp->lines.capacity = PUSH_LABEL_SCOPE_LIMIT;
		comp->lines.changed = pool_request(comp->mem, sizeof(byte)*PUSH_LABEL_SCOPE_LIMIT);
	}
	comp->lines.map[0] = loc_thunk_map_init(comp->mem);
	comp->lines.line[0] = 0;
	comp->lines.changed[0] = 0;
	pool subline_pool = pool_alloc(AUX_SIZE, POOL_STATIC);
	ltms sublines = {
		.map = pool_request(comp->mem, sizeof(loc_thunk_map)*PUSH_LABEL_SCOPE_LIMIT),
		.line = pool_request(comp->mem, sizeof(word)*PUSH_LABEL_SCOPE_LIMIT),
		.size = 1,
		.capacity = PUSH_LABEL_SCOPE_LIMIT,
		.changed = pool_request(comp->mem, sizeof(byte)*PUSH_LABEL_SCOPE_LIMIT)
	};
	sublines.map[0] = loc_thunk_map_init(&subline_pool);
	sublines.line[0] = 0;
	sublines.changed[0] = 0;
	pregenerate(comp, &sublines, comp->ir);
	ASSERT_LOCAL(comp->lines.size == 1 && sublines.size == 1, " loc_thunk_map stack corrupted\n");
	while (comp->lines.changed[0] || sublines.changed[0]){
		pool_empty(&subline_pool);
		loc_thunk_map_empty(&sublines.map[0]);
		loc_thunk_map_empty(&comp->lines.map[0]);
		sublines.line[0] = 0;
		sublines.changed[0] = 0;
		comp->lines.line[0] = 0;
		comp->lines.changed[0] = 0;
		correct_offsets(comp, &sublines, comp->ir);
	}
	ASSERT_LOCAL(comp->lines.size == 1, " ended program in nested push\n");
	pool_dealloc(&subline_pool);
	comp->buf = pool_request(comp->mem, 4*comp->lines.line[0]);
	return 0;
}

void
generate_code(compiler* const comp){
	word i = 0;
	code_tree* block = comp->ir;
	while (block != NULL){
		word n = block->code.instruction_count*4;
		for (word k = 0;k<n;++k){
			comp->buf[i++] = block->code.instructions[k];
		}
		block = block->next;
	}
}

byte
flatten_macro_definitions(compiler* const comp){
	pool* aux = pool_request(comp->mem, sizeof(pool));
	*aux = pool_alloc(AUX_SIZE, POOL_STATIC);
	while (find_macros(comp, aux) == 1){
		replace_macros(comp, aux);
		if (*comp->err != 0){
			break;
		}
#ifdef ORB_DEBUG
		show_tokens(comp);
#endif
		pool* temp = comp->tok;
		comp->tok = comp->tok_swp;
		comp->tok_swp = temp;
		pool_empty(comp->tok_swp);
		pool_empty(aux);
		macro_map_empty(&comp->macros);
		comp->macro_calls = NULL;
	}
	pool_dealloc(aux);
	return 0;
}

//TODO optimization pass
byte
compile_cstr(compiler* const comp, byte noentry){
	lex_cstr(comp, 0, noentry);
	ASSERT_ERR(0);
#ifdef ORB_DEBUG
	show_tokens(comp);
	printf("\033[1;42mFLATTENING MACROS:\033[0m");
#endif
	flatten_macro_definitions(comp);
	ASSERT_ERR(0);
#ifdef ORB_DEBUG
	show_tokens(comp);
	printf("\033[1;42mPARSING:\033[0m");
#endif
	parse_tokens(comp);
	ASSERT_ERR(0);
#ifdef ORB_DEBUG
	show_block(comp, comp->ir, 0);
	printf("\033[1;42mPRE-GENERATING:\033[0m");
#endif
	backpass(comp);
	ASSERT_ERR(0);
#ifdef ORB_DEBUG
	show_block(comp, comp->ir, 0);
	printf("\033[1;42mDONE\033[0m\n");
#endif
	generate_code(comp);
	return 0;
}

void
compile_file(char* infile, char* outfile, byte noentry){
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
	word read_bytes = fread(mem.buffer, sizeof(byte), mem.left, fd);
	fclose(fd);
	if (read_bytes == mem.left){
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
	OPCODE_map opmap = OPCODE_map_init(&mem);
	REGISTER_map regmap = REGISTER_map_init(&mem);
	REG_PARTITION_map partmap = REG_PARTITION_map_init(&mem);
	EXTERNAL_CALLS_map extmap = EXTERNAL_CALLS_map_init(&mem);
	char_map inclusions = char_map_init(&mem);
	macro_map macros = macro_map_init(&mem);
	setup_opcode_map(&opmap);
	setup_register_map(&regmap);
	setup_partition_map(&partmap);
	setup_external_call_map(&extmap);
	pool code = pool_alloc(WRITE_BUFFER_SIZE, POOL_STATIC);
	pool tok = pool_alloc(READ_BUFFER_SIZE, POOL_STATIC);
	pool tok_swp = pool_alloc(READ_BUFFER_SIZE, POOL_STATIC);
	compiler comp = {
		.str = str,
		.opmap = opmap,
		.regmap = regmap,
		.partmap = partmap,
		.extmap = extmap,
		.inclusions = inclusions,
		.macros = macros,
		.macro_calls = NULL,
		.mem = &mem,
		.code = &code,
		.tok = &tok,
		.tok_swp = &tok_swp,
		.buf = NULL,
		.err = pool_request(&mem, ERROR_BUFFER)
	};
	*comp.err = 0;
	compile_cstr(&comp, noentry);
	if (*comp.err != 0){
		fprintf(stderr, "Unable to compile '%s'\n", infile);
		fprintf(stderr, comp.err);
		fprintf(stderr, "\033[0m\n");
		pool_dealloc(&mem);
		pool_dealloc(&code);
		pool_dealloc(&tok);
		pool_dealloc(&tok_swp);
		return;
	}
	fd = fopen(outfile, "w");
	if (fd == NULL){
		fprintf(stderr, "Unable to open file '%s' for writing\n", outfile);
		pool_dealloc(&mem);
		pool_dealloc(&code);
		pool_dealloc(&tok);
		pool_dealloc(&tok_swp);
		return;
	}
	word bytes = 4*comp.lines.line[0];
	if (fwrite(comp.buf, 1, bytes, fd) < bytes){
		fprintf(stderr, "Unable to write to output file\n");
	}
	fclose(fd);
	pool_dealloc(&mem);
	pool_dealloc(&code);
	pool_dealloc(&tok);
	pool_dealloc(&tok_swp);
}

void
setup_registers(machine* const mach){
	for (uint8_t r = 0;r<REGISTER_COUNT;++r){
		for (uint8_t i = 0;i<4;++i){
			mach->quar[(r*4)+i] = (uint16_t*)(&mach->reg[r])+(3-i);
		}
		mach->half[r] = (uint32_t*)(&mach->reg[r]);
		mach->lo[r] = (byte*)(&mach->reg[r]);
		mach->hi[r] = (byte*)(&mach->reg[r])+1;
	}
	mach->reg[SP] = MEMORY_SIZE-1;
	mach->reg[FP] = MEMORY_SIZE-1;
	mach->reg[CR] = MEMORY_SIZE-1;
	mach->reg[IP] = 0;
}

void
flash_rom(machine* const mach, byte* buffer, uint64_t size){
	for (uint64_t i = 0;i<size;++i){
		mach->mem[PROGRAM_START+i] = buffer[i];
	}
	mach->mem_ptr = size + PROGRAM_START;
}

void
demo(){
	machine mach;
	setup_machine(&mach);
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
	flash_rom(&mach, cc, 128);
	interpret(&mach, 1);
	return;
}

void
show_binary(char* filename){
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

void
poll_input(machine* const mach){
	while (SDL_PollEvent(&mach->event)){
		if (mach->event.type == SDL_KEYDOWN){
			mach->dev[KEYBOARD_DEVICE][mach->event.key.keysym.scancode] = 1;
		}
		else if (mach->event.type == SDL_KEYUP){
			mach->dev[KEYBOARD_DEVICE][mach->event.key.keysym.scancode] = 0;
		}
	}
}

void
setup_devices(machine* const mach){
	for (byte i = 0;i<DEVICE_COUNT;++i){
		mach->dev[i] = NULL;
	}
	mach->dev[KEYBOARD_DEVICE] = mach->keys;
	memset(mach->keys, 0, 256);
	mach->dev[SCREEN_DEVICE] = (byte*)mach->frame_buffer;
}

void
run_rom(char* filename, byte debug){
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
	machine mach;
	setup_machine(&mach);
	flash_rom(&mach, buffer, size);
	interpret(&mach, debug);
	free(buffer);
	SDL_DestroyWindow(mach.window);
	SDL_DestroyRenderer(mach.renderer);
	SDL_Quit();
}

void
setup_machine(machine* const mach){
	mach->mem_ptr = PROGRAM_START;
	mach->aux_ptr = AUX_MEM_START;
	mach->prog_ptr = PROG_MEM_START;
	setup_registers(mach);
	SDL_CreateWindowAndRenderer(SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_OPENGL, &mach->window, &mach->renderer);
	SDL_SetWindowTitle(mach->window, "Orb");
	SDL_Init(SDL_INIT_EVERYTHING);
	setup_devices(mach);
}

int32_t
main(int argc, char** argv){
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
		if (argc < 5){
			printf(" Wrong number of arguments for compilation: -c infile.src -o outfile.rom [optional: -noentry]\n");
			return 0;
		}
		if (strncmp(argv[3], "-o", TOKEN_MAX) != 0){
			printf(" Compilation requires '-o' to designate output file name\n");
			return 0;
		}
		if (argc < 6){
			compile_file(argv[2], argv[4], 0);
		}
		else if (strncmp(argv[5], "-noentry", TOKEN_MAX) != 0){
			compile_file(argv[2], argv[4], 1);
		}
		return 0;
	}
	if (strncmp(argv[1], "-r", TOKEN_MAX) == 0){
		if (argc < 3){
			printf(" Needs a rom image name\n");
			return 0;
		}
		if (argc < 4){
			run_rom(argv[2], 0);
		}
		else if (strncmp(argv[3], "-g", TOKEN_MAX) == 0){
			run_rom(argv[2], 1);
		}
		else {
			printf(" Unknopwn flag option '%s'\n", argv[3]);
		}
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

