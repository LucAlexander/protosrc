
```asm

main:
    "hello world\n"
    MOV rA SP
    ADS rA 0x1
    LDS rB 0xC
    EXT OUT
    REI

```

```
 ,---REGISTERS--------------------------------------.
 |  General  |  Special  |  Purpose                 |
 :-----------+-----------+--------------------------:
 |  rA       |  IP       |  Instruction Pointer     |
 |  rB       |  SP       |  Stack Pointer           |
 |  rC       |  FP       |  Frame Pointer           |
 |  rD       |  SR       |  Status Register         | <---.---BITS---
 |  rE       |  CR       |  Call Argument Register  |     | 0 : Zero
 |  rF       |  AR       |  Aux Register            |     | 1 : Carry
 |  rG       :-----------'--------------------------'     | 2 : Sign
 |  rH       |    ^                                       | 3 : Over
 |  rI       |    '--[ These are preserved between calls  '---------
 '-----------'

 ,---PARTITIONS---------------------------------------------------.
 |  Q                                                             | <-[ `rA Q` is the same as just `rA`
 |::::::::::::::::::::::::::::::::  D                             |
 |  Z            :::::::::::::::::::::::::::::::::::::::::::::::::|
 |::::::::::::::::  Y            :::::::::::::::::::::::::::::::::|
 |::::::::::::::::::::::::::::::::  X            :::::::::::::::::|
 |::::::::::::::::::::::::::::::::::::::::::::::::  W             |
 |::::::::::::::::::::::::::::::::::::::::::::::::  HI   :::::::::|
 |::::::::::::::::::::::::::::::::::::::::::::::::::::::::  LO    |
 '----------------------------------------------------------------'

 ,---INSTRUCTIONS----------------------.-------------------.
 |  opcode  |  arguments               |  example          |
 :----------+--------.--------.--------+-------------------:
 |  byte    |  byte  |  byte  |  byte  |                   |
 :----------+--------+--------+--------+-------------------:
 |  NOP     |  0     |  0     |  0     |   NOP             |
 |  LDS     |  dest  |  short literal  |   LDS rA 0xFACE   |
 |  LDA     |  dest  |  addr  |  off   |   LDA rA rB rC    |
 |  LDI     |  dest  |  addr  |  lit   |   LDI rA rB 0x8   |
 |  STS     |  src   |  short literal  |   STS rB 0xCAB    |
 |  STA     |  src   |  addr  |  off   |   STA rB rC rD    |
 |  STB     |  src   |  addr  |  lit   |   STB rB rC 0x9   |
 |  MOV     |  dst   |  src   |  0     |   MOV rC rD       |
 |  SWP     |  dst   |  src   |  0     |   SWP rC rD       |
 |  ADS     |  dest  |  short literal  |   ADS rD 0xCAFE   |
 |  SUS     |  dest  |  short literal  |                   |
 |  MUS     |  dest  |  short literal  |                   |
 |  DIS     |  dest  |  short literal  |                   |
 |  MOS     |  dest  |  short literal  |                   |
 |  ANS     |  dest  |  short literal  |                   |
 |  ORS     |  dest  |  short literal  |                   |
 |  SLS     |  dest  |  short literal  |                   |
 |  SRS     |  dest  |  short literal  |                   |
 |  XRS     |  dest  |  short literal  |                   |
 |  ADI     |  dst   |  right |  0     |   ADI rD rE       |
 |  SUI     |  dst   |  right |  0     |                   |
 |  MUI     |  dst   |  right |  0     |                   |
 |  DII     |  dst   |  right |  0     |                   |
 |  MOI     |  dst   |  right |  0     |                   |
 |  ANI     |  dst   |  right |  0     |                   |
 |  ORI     |  dst   |  right |  0     |                   |
 |  SLI     |  dst   |  right |  0     |                   |
 |  SRI     |  dst   |  right |  0     |                   |
 |  XRI     |  dst   |  right |  0     |                   |
 |  ADD     |  dst   |  left  |  right |   ADD rD rE rF    |
 |  SUB     |  dst   |  left  |  right |                   |
 |  MUL     |  dst   |  left  |  right |                   |
 |  DIV     |  dst   |  left  |  right |                   |
 |  MOD     |  dst   |  left  |  right |                   |
 |  AND     |  dst   |  left  |  right |                   |
 |  OR      |  dst   |  left  |  right |                   |
 |  SHL     |  dst   |  left  |  right |                   |
 |  SHR     |  dst   |  left  |  right |                   |
 |  XOR     |  dst   |  left  |  right |                   |
 |  INV     |  dst   |  src   |  0     |   INV rE rF       |
 |  COM     |  dst   |  src   |  0     |   COM rE rF       |
 |  INI     |  dst   |  0     |  0     |   INI rF          |
 |  COI     |  dst   |  0     |  0     |   COI rF          |
 |  CMP     |  left  |  right |  0     |   CMP rA rB       |
 |  CMS     |  left  |  short literal  |   CMS rA 0xBEEF   |
 |  RET     |  tar   |  0     |  0     |   RET rB          |
 |  REI     |  0     |  0     |  0     |   REI             |
 |  RES     |  short literal  |  0     |   RES 0xDEAD      |
 |  CAL     |  0     |  0     |  0     |   CAL             |
 |  PSH     |  tar   |  0     |  0     |   PSH rC          |
 |  PSS     |  short literal  |  0     |   PSS 0xAFFE      |
 |  POP     |  dst   |  0     |  0     |   POP rD          |
 |  BNC     |  mode  |  short literal  |   B**             | --.
 |  BNE     |  mode  |  short literal  |   B** label       |   |
 |  BEQ     |  mode  |  short literal  |   B** .sub        |   |      ,--MODES--
 |  BLT     |  mode  |  short literal  |   B** 0xB         |   |      |
 |  BGT     |  mode  |  short literal  |   B** -0xC        |   |      | 0: jump by relative
 |  BLE     |  mode  |  short literal  |   B** LR          |   |      |    register stored
 |  BGE     |  mode  |  short literal  |                   |   :------:    offset
 |  JMP     |  mode  |  short literal  |   J**             |   |      | 1: jump by relative
 |  JNE     |  mode  |  short literal  |   J** label       |   |      |    short literal
 |  JEQ     |  mode  |  short literal  |   J** .sub        |   |      |    offset
 |  JLT     |  mode  |  short literal  |   J** 0xD         |   |      | 2: jump to register
 |  JGT     |  mode  |  short literal  |   J** -0xF        |   |      |    stored addtess
 |  JLE     |  mode  |  short literal  |   J** LR          |   |      |
 |  JGE     |  mode  |  short literal  |                   | --'      '---
 |  EXT     |  call  |  0     |  0     |   EXT OUT         |
 |  EXR     |  reg   |  0     |  0     |   EXR rE          |
 '----------'--------'--------'--------'-------------------'
```

```asm
(procedure 0xCAFE rA {0xDEAFBEEFFACECAFE}) 

;evaluates to

CAL
PSS 0xCAFE 
PSH rA
{0xDEADBEEFFACECAFE}
BNC procedure

```

