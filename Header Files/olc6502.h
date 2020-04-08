#pragma once
#include <cstdint>
#include <vector>

// 6502 Datasheet: http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf


class Bus; //Forward declare the bus

class olc6502{

    public:
        olc6502();
        ~olc6502();

    
        enum FLAGS6502{
        //  00000000
        //  NVUBDIZC
            C = (1 << 0), // Carry bit
            Z = (1 << 1), // Zero
            I = (1 << 2), // Disable Interrupts
            D = (1 << 3), // Decimal Mode (unused)
            B = (1 << 4), // Break
            U = (1 << 5), // Unused
            V = (1 << 6), // Overflow
            N = (1 << 7), // Negative
        };

        uint8_t a = 0x00; // Accumulator Register
        uint8_t x = 0x00; // X Register
        uint8_t y = 0x00; // Y Register
        uint8_t stkp = 0x00; // Stack Pointer
        uint16_t pc = 0x00; // Program Counter
        uint8_t status = 0x00; // Status Register

        void ConnectBus(Bus *n) { bus = n; }



        // Used to catch illegal opcodes
        uint8_t XXX();

        // Alert cpu that a clock cycle has occured
        void clock();

// ----------------------------------------------------------------------------------------------
        // All three below need to work asynchronously and will interrupt CPU at anytime
        // Note: CPU will finish current instruction in execution even when these are called

        // Note: Standard interrupts can be ignored if interrupt enable flag is set or not,
        //       the non-maskable cannot
        void reset();
        // Interrupt request signal
        void irq();
        // Non-maskable inturrupt request signal
        void nmi();
// ----------------------------------------------------------------------------------------------

    private:
        Bus *bus = nullptr;
        uint8_t read(uint16_t a);
        void write(uint16_t a, uint8_t d);

        // Status register accessors, these will wrap the bitwise operations
        uint8_t GetFlag(FLAGS6502 f);
        void    SetFlag(FLAGS6502 f, bool v);
        uint8_t fetch(); //internal data helper


        uint8_t fetched = 0x00; // where internal data is to be stored. Working input value to ALU
        uint16_t addr_abs = 0x0000; // All used memory addresses
        uint16_t addr_rel = 0x00; // Absolute address following a branch instruction. (branch inst can only go so far from where called on 6502)
        uint8_t opcode = 0x00;
        uint8_t cycles = 0; // Cycles left for current instruction


        // Opcodes (56) found here: http://obelisk.me.uk/6502/reference.html
        // Operations to be performed on operands (data)
        uint8_t ADC();	uint8_t AND();	uint8_t ASL();	uint8_t BCC();
	    uint8_t BCS();	uint8_t BEQ();	uint8_t BIT();	uint8_t BMI();
    	uint8_t BNE();	uint8_t BPL();	uint8_t BRK();	uint8_t BVC();
    	uint8_t BVS();	uint8_t CLC();	uint8_t CLD();	uint8_t CLI();
    	uint8_t CLV();	uint8_t CMP();	uint8_t CPX();	uint8_t CPY();
    	uint8_t DEC();	uint8_t DEX();	uint8_t DEY();	uint8_t EOR();
    	uint8_t INC();	uint8_t INX();	uint8_t INY();	uint8_t JMP();
    	uint8_t JSR();	uint8_t LDA();	uint8_t LDX();	uint8_t LDY();
    	uint8_t LSR();	uint8_t NOP();	uint8_t ORA();	uint8_t PHA();
    	uint8_t PHP();	uint8_t PLA();	uint8_t PLP();	uint8_t ROL();
    	uint8_t ROR();	uint8_t RTI();	uint8_t RTS();	uint8_t SBC();
    	uint8_t SEC();	uint8_t SED();	uint8_t SEI();	uint8_t STA();
    	uint8_t STX();	uint8_t STY();	uint8_t TAX();	uint8_t TAY();
    	uint8_t TSX();	uint8_t TXA();	uint8_t TXS();	uint8_t TYA();

        // 12 addressing modes found here: http://wiki.nesdev.com/w/index.php/CPU_addressing_modes
        // Essentially tells where the data (operands) can be found
        uint8_t IMP();
        uint8_t ZPO();
        uint8_t ZPY();
        uint8_t ABS();
        uint8_t ABY();
        uint8_t IZX();
        uint8_t IMM();
        uint8_t ZPX();
        uint8_t REL();
        uint8_t ABX();
        uint8_t IND();
        uint8_t IZY();

        struct INSTRUCTION {
            std::string name;
            uint8_t(olc6502::*operate)(void) = nullptr; // Function pointer to operation to be performed
            uint8_t(olc6502::*addrmode)(void) = nullptr; // Function pointer to address mode
            uint8_t     cycles = 0; // Count of clock cycles for instruction
        };

        std::vector<INSTRUCTION> lookup;

};