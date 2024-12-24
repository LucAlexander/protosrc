#ifndef ORB_H
#define ORB_H

#include <inttypes.h>
#include "string.h"
#include "hashmap.h"

//#define ORB_DEBUG

#define READ_BUFFER_SIZE 0x10000000
#define WRITE_BUFFER_SIZE 0x10000000
#define AUX_SIZE 0x100000
#define ERROR_BUFFER 0x100
#define ARENA_SIZE READ_BUFFER_SIZE+AUX_SIZE+ERROR_BUFFER
#define TOKEN_MAX 64

typedef uint64_t word;
typedef uint8_t byte;

typedef enum {
	NOP=0,
	LDS, LDA, LDI,
	STS, STA, STB,
	MOV, SWP,
	ADS, SUS, MUS, DIS, MOS, ANS, ORS, SLS, SRS, XRS,
	ADI, SUI, MUI, DII, MOI, ANI, ORI, SLI, SRI, XRI,
	ADD, SUB, MUL, DIV, MOD, AND, OR, SHL, SHR, XOR,
	INV, COM, INI, COI,
	CMP, CMS,
	RET, REI, RES,
	CAL,
	PSH, PSS,
	POP,
	BNC, BNE, BEQ, BLT, BGT, BLE, BGE,
	JMP, JNE, JEQ, JLT, JGT, JLE, JGE,
	INT, INR,
	OPCODE_COUNT
} OPCODE;

MAP_DEF(OPCODE)

typedef enum {
	IP, SP, FP, SR, LR, CR, AR,
	R0, R1, R2, R3, R4, R5, R6, R7, R8,
	REGISTER_COUNT
} REGISTER;

MAP_DEF(REGISTER)

typedef enum {
	L16,
	LM16,
	RM16,
	R16,
	FULL,
	HALF,
	LO,
	HI,
	PARTITION_COUNT
} REG_PARTITION;

MAP_DEF(REG_PARTITION)

typedef enum {
	ZERO=1,
	CARRY=2,
	SIGN=4,
	OVER=8
} STATUS_FLAG;

typedef struct {
	char* text;
	word size;
	word i;
} string;

typedef enum {
	OPCODE_TOKEN,
	REGISTER_TOKEN,
	PART_TOKEN,
	OPEN_CALL_TOKEN='(',
	CLOSE_CALL_TOKEN=')',
	OPEN_PUSH_TOKEN='{',
	CLOSE_PUSH_TOKEN='}',
	SUBLABEL_TOKEN='.',
	LABEL_TOKEN=':',
	SHORT_HEX_NUMERIC_TOKEN,
	BYTE_HEX_NUMERIC_TOKEN,
	IDENTIFIER_TOKEN,
	NONE_TOKEN
} TOKEN;

typedef struct {
	char* text;
	union {
		int64_t number;
		OPCODE opcode;
		REGISTER reg;
		REG_PARTITION part;
	} data;
	TOKEN type;
	byte size;
} token;

typedef struct code_tree code_tree;
typedef struct data_tree data_tree;
typedef struct call_tree call_tree;
typedef struct block_scope block_scope;

typedef struct vm_code {
	byte* instructions;
	word instruction_count;
} vm_code;

typedef struct data_tree {
	union {
		struct {
			byte* raw;
			word size;
		} bytes;
		data_tree* nest;
		code_tree* code;
	} data;
	data_tree* next;
	enum {
		BYTE_DATA,
		NEST_DATA,
		CODE_DATA
	} type;
} data_tree;

typedef struct call_tree {
	union {
		call_tree* call;
		data_tree* push;
		byte reg;
		struct {
			token label;
			code_tree* dest_block;
		} labeling;
		int64_t number;
	} data;
	call_tree* next;
	enum {
		CALL_ARG,
		PUSH_ARG,
		REG_ARG,
		LABEL_ARG,
		SUBLABEL_ARG,
		NUMERIC_ARG
	} type;
} call_tree;

#define BLOCK_START_SIZE 8

typedef struct code_tree {
	token label;
	token dest;
	vm_code code;
	union {
		data_tree* push;
		call_tree* call;
	} data;
	code_tree* dest_block;
	code_tree* next;
	enum {
		INSTRUCTION_BLOCK,
		INSTRUCTION_JUMP,
		INSTRUCTION_SUBJUMP,
		CALL_BLOCK,
		PUSH_BLOCK
	} type;
	enum {
		NOT_LABELED,
		LABELED,
		SUBLABELED
	} labeling;
} code_tree;

typedef struct block_scope {
	token pending_source;
	block_scope* next;
	code_tree* label;
	code_tree** ref;
	pool* mem;
	enum {
		FULFILLED_MEMBER,
		PENDING_MEMBER
	} type;
} block_scope;

MAP_DEF(block_scope)

#define PUSH_LABEL_SCOPE_LIMIT 4

typedef struct bsms {
	block_scope_map* map;
	byte size;
	byte capacity;
} bsms;

byte block_scope_add_member(block_scope_map* const block, token t, code_tree* member);
code_tree* block_scope_check_member(block_scope_map* const block, token t, code_tree** ref);
void bsms_push(bsms* stack);
void bsms_pop(bsms* stack);

typedef struct {
	string str;
	string buf;
	OPCODE_map opmap;
	REGISTER_map regmap;
	REG_PARTITION_map partmap;
	token* tokens;
	word token_count;
	code_tree* ir;
	bsms labels;
	pool* mem;
	pool* code;
	char* err;
} compiler;

void show_registers();
void show_mem();
void show_machine();
void interpret();
void setup_opcode_map(OPCODE_map* opmap);
void setup_register_map(REGISTER_map* regmap);
void setup_partition_map(REG_PARTITION_map* partmap);
byte whitespace(char c);
byte lex_cstr(compiler* const comp);
word parse_register(compiler* const comp, word token_index, byte* r);
word parse_call_block(compiler* const comp, bsms* const sublabels, word token_index, call_tree* data);
word parse_byte_sequence(compiler* const comp, word token_index, data_tree* data);
word parse_push_block(compiler* const comp, bsms* const sublabels, word token_index, data_tree* data);
word parse_3reg(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code);
word parse_2reg(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code);
word parse_2reg_byte(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code);
word parse_reg_short(compiler* const comp, OPCODE op, word instruction_index, word token_index, code_tree* code);
word parse_instruction_block(compiler* const comp, bsms* const sublabels, word token_index, code_tree* code);
word parse_code(compiler* const comp, bsms* const sublabels, word token_index, code_tree* ir, TOKEN terminator);
byte parse_tokens(compiler* const comp);
byte show_call(compiler* const comp, call_tree* const data, word depth);
byte show_data(compiler* const comp, data_tree* const data, word depth);
byte show_block(compiler* const comp, code_tree* const code, word depth);
void show_tokens(compiler* const comp);
byte compile_cstr(compiler* const comp);
void compile_file(char* infile, char* outfile);
void setup_registers();
void flash_rom(byte* buffer, uint64_t size);
void demo();
void show_binary(char* filename);
void run_rom(char* filename);
byte check_label_bucket(compiler* const comp, block_scope_map_bucket* bucket);
byte remaining_labels(compiler* const comp, block_scope_map* const block);
code_tree* pregen_push(compiler* const comp, code_tree* basic_block, data_tree* push);
code_tree* pregen_call(compiler* const comp, code_tree* basic_block, call_tree* call);
byte pregenerate(compiler* const comp, code_tree* basic_block);

#endif
