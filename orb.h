#ifndef ORB_H
#define ORB_H

#include <inttypes.h>
#include "string.h"
#include "hashmap.h"

//#define ORB_DEBUG

#define READ_BUFFER_SIZE 0x10000000
#define WRITE_BUFFER_SIZE 0x10000000
#define MACHINE_AUX_SIZE 0x10000000
#define AUX_SIZE 0x100000
#define ERROR_BUFFER 0x100
#define ARENA_SIZE READ_BUFFER_SIZE+AUX_SIZE+ERROR_BUFFER
#define TOKEN_MAX 64

#define INSTRUCTION_WIDTH 0x4

#define PRELUDE_SIZE 0x1000
#define PROGRAM_START PRELUDE_SIZE
#define PROGRAM_SIZE 0x100000

#define MAIN_MEM_START PROGRAM_SIZE + PRELUDE_SIZE
#define MAIN_MEM_SIZE 0x100000

#define MEMORY_SIZE MAIN_MEM_SIZE + PROGRAM_SIZE + PRELUDE_SIZE

#define PROG_MEM_START MEMORY_SIZE
#define PROG_MEM_SIZE 0x10000

#define AUX_MEM_START PROG_MEM_START + PROG_MEM_SIZE
#define AUX_MEM_SIZE 0x100000

#define FULL_MEM_SIZE AUX_MEM_START + AUX_MEM_SIZE

#define DEVICE_COUNT 0x8

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
	EXT, EXR,
	OPCODE_COUNT
} OPCODE;

MAP_DEF(OPCODE)

/* ARGUMENTS
 *  OUT      a : string address
 *           b : string length
 *  END
 *  MEM      a : request size -> a : address
 *  MEM_PROG a : request size -> a : address
 *  MEM_AUX  a : request size -> a : address
 *
*/

typedef enum {
	EXT_OUT,
	EXT_END,
	EXT_MEM,
	EXT_MEM_PROG,
	EXT_MEM_AUX,
	EXT_COUNT
} EXTERNAL_CALLS;

MAP_DEF(EXTERNAL_CALLS)

typedef enum {
	IP, SP, FP, SR, LR, CR, AR,
	R0, R1, R2, R3, R4, R5, R6, R7, R8,
	REGISTER_COUNT
} REGISTER;

typedef struct machine {
	word reg[REGISTER_COUNT];
	uint16_t* quar[REGISTER_COUNT*4];
	uint32_t* half[REGISTER_COUNT];
	byte* lo[REGISTER_COUNT];
	byte* hi[REGISTER_COUNT];
	byte mem[FULL_MEM_SIZE];
	byte* dev[DEVICE_COUNT];
	word mem_ptr;
	word aux_ptr;
	word prog_ptr;
} machine;

void setup_machine(machine* const mach);
void setup_devices(machine* const mach);
void setup_registers(machine* const mach);

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
	EXT_TOKEN,
	SHORT_HEX_NUMERIC_TOKEN,
	BYTE_HEX_NUMERIC_TOKEN,
	DWORD_HEX_NUMERIC_TOKEN,
	QWORD_HEX_NUMERIC_TOKEN,
	IDENTIFIER_TOKEN,
	OPEN_MACRO_TOKEN='[',
	CLOSE_MACRO_TOKEN=']',	
	MACRO_EVAL_TOKEN='=',
	OPEN_CALL_TOKEN='(',
	CLOSE_CALL_TOKEN=')',
	OPEN_PUSH_TOKEN='{',
	CLOSE_PUSH_TOKEN='}',
	STRING_TOKEN='"',
	INCLUDE_TOKEN='+',
	SUBLABEL_TOKEN='.',
	LABEL_TOKEN=':',
	NONE_TOKEN
} TOKEN;

typedef struct {
	char* text;
	union {
		int64_t number;
		OPCODE opcode;
		REGISTER reg;
		REG_PARTITION part;
		EXTERNAL_CALLS ext;
	} data;
	TOKEN type;
	byte size;
} token;

typedef struct code_tree code_tree;
typedef struct data_tree data_tree;
typedef struct call_tree call_tree;
typedef struct macro_tree macro_tree;
typedef struct block_scope block_scope;

typedef struct vm_code {
	byte* instructions;
	word instruction_count;
} vm_code;

typedef enum {
	CALL_ARG,
	PUSH_ARG,
	REG_ARG,
	LABEL_ARG,
	SUBLABEL_ARG,
	NUMERIC_ARG
} ARG_TYPE;

typedef struct macro_def {
	union {
		code_tree* code;
		data_tree* push;
		call_tree* call;
		struct {
			token label;
			code_tree* dest_block;
		} labeling;
		int64_t number;
		byte reg;
	} data;
	ARG_TYPE type;
} macro_def;

MAP_DEF(macro_def)

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
		CODE_DATA,
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
	ARG_TYPE type;
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
	code_tree* prev;
	enum {
		INSTRUCTION_BLOCK,
		INSTRUCTION_JUMP,
		INSTRUCTION_SUBJUMP,
		CALL_BLOCK,
		PUSH_BLOCK,
	} type;
	enum {
		NOT_LABELED,
		LABELED,
		SUBLABELED
	} labeling;
} code_tree;

typedef enum THUNK_MEMBER {
	FULFILLED_MEMBER,
	PENDING_MEMBER
} THUNK_MEMBER;

typedef struct block_scope {
	token pending_source;
	block_scope* next;
	code_tree* label;
	code_tree** ref;
	pool* mem;
	THUNK_MEMBER type;
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

typedef struct loc_thunk loc_thunk;

typedef struct loc_thunk {
	loc_thunk* next;
	token label; // pending + fulfilled label
	word line; // fulfilled line
	word jump_line; // pending line
	code_tree* jump; // pending jump
	byte (*f)(code_tree* jump, word jumpline, word line);
	THUNK_MEMBER type;
} loc_thunk;

MAP_DEF(loc_thunk)

typedef struct ltms {
	loc_thunk_map* map;
	word* line; // current line of generated code in pass
	byte size;
	byte capacity;
	byte* changed;
} ltms;

byte replace_call_arg(code_tree* jump, word jumpline, word line);
byte replace_call_dest(code_tree* jump, word jumpline, word line);
byte loc_thunk_add_member(ltms* const stack, token t);
void loc_thunk_check_member(ltms* const stack, token t, byte(*f)(code_tree*, word, word), code_tree* ref);
void ltms_push(ltms* stack);
void ltms_pop(ltms* stack);

byte register_macro_arg(macro_arg_map* const map, macro_arg arg);

MAP_DEF(word)

typedef struct {
	string str;
	OPCODE_map opmap;
	REGISTER_map regmap;
	REG_PARTITION_map partmap;
	EXTERNAL_CALLS_map extmap;
	token* tokens;
	word token_count;
	code_tree* ir;
	bsms labels;
	ltms lines;
	macro_tree_map* args;
	word_map* macro_defs;
	pool* mem;
	pool* code;
	pool* tok;
	byte* buf;
	char* err;
} compiler;

void show_registers(machine* const mach);
void show_mem(machine* const mach);
void show_machine(machine* const mach);
void interpret(machine* const mach, byte debug);
byte interpret_external(machine* const mach, byte ext);
void setup_opcode_map(OPCODE_map* opmap);
void setup_register_map(REGISTER_map* regmap);
void setup_partition_map(REG_PARTITION_map* partmap);
byte whitespace(char c);
byte lex_cstr(compiler* const comp, byte nested);
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
void flash_rom(machine* const mach, byte* buffer, uint64_t size);
void demo();
void show_binary(char* filename);
void run_rom(char* filename, byte debug);
byte check_label_bucket(compiler* const comp, block_scope_map_bucket* bucket);
byte remaining_labels(compiler* const comp, block_scope_map* const block);
code_tree* pregen_push(compiler* const comp, ltms* const sublines, code_tree* basic_block, data_tree* push);
code_tree* pregen_call(compiler* const comp, ltms* const sublines, code_tree* basic_block, call_tree* call);
void pregenerate(compiler* const comp, ltms* const sublines, code_tree* basic_block);
void correct_offsets(compiler* const comp, ltms* const sublines, code_tree* basic_block);

#endif
