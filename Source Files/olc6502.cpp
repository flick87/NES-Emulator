#pragma once

#include "olc6502.h"
#include "Bus.h"

// struct INSTRUCTION {
//             std::string name;
//             uint8_t(olc6502::*operate)(void) = nullptr; // Function pointer to operation to be performed
//             uint8_t(olc6502::*addrmode)(void) = nullptr; // Function pointer to address mode to manipulate data
//             uint8_t     cycles = 0; // Count of clock cycles for instruction
// };

olc6502::olc6502(){

    using a = olc6502; // NOT ACCUMULATOR a

	// *Note: Not all but many opcodes will have 16 entries for each of the 16 addressing modes
	// List copied directly from https://github.com/OneLoneCoder/olcNES/blob/master/Part%232%20-%20CPU/olc6502.cpp
	// List corresponds to datasheet matrix (pg. 10): http://archive.6502.org/datasheets/rockwell_r650x_r651x.pdf
	lookup =  
	{
		{ "BRK", &a::BRK, &a::IMM, 7 },{ "ORA", &a::ORA, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::ZPO, 3 },{ "ASL", &a::ASL, &a::ZPO, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHP", &a::PHP, &a::IMP, 3 },{ "ORA", &a::ORA, &a::IMM, 2 },{ "ASL", &a::ASL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABS, 4 },{ "ASL", &a::ASL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BPL", &a::BPL, &a::REL, 2 },{ "ORA", &a::ORA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ZPX, 4 },{ "ASL", &a::ASL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLC", &a::CLC, &a::IMP, 2 },{ "ORA", &a::ORA, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ORA", &a::ORA, &a::ABX, 4 },{ "ASL", &a::ASL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "JSR", &a::JSR, &a::ABS, 6 },{ "AND", &a::AND, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "BIT", &a::BIT, &a::ZPO, 3 },{ "AND", &a::AND, &a::ZPO, 3 },{ "ROL", &a::ROL, &a::ZPO, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLP", &a::PLP, &a::IMP, 4 },{ "AND", &a::AND, &a::IMM, 2 },{ "ROL", &a::ROL, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "BIT", &a::BIT, &a::ABS, 4 },{ "AND", &a::AND, &a::ABS, 4 },{ "ROL", &a::ROL, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BMI", &a::BMI, &a::REL, 2 },{ "AND", &a::AND, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ZPX, 4 },{ "ROL", &a::ROL, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEC", &a::SEC, &a::IMP, 2 },{ "AND", &a::AND, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "AND", &a::AND, &a::ABX, 4 },{ "ROL", &a::ROL, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "RTI", &a::RTI, &a::IMP, 6 },{ "EOR", &a::EOR, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "EOR", &a::EOR, &a::ZPO, 3 },{ "LSR", &a::LSR, &a::ZPO, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PHA", &a::PHA, &a::IMP, 3 },{ "EOR", &a::EOR, &a::IMM, 2 },{ "LSR", &a::LSR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::ABS, 3 },{ "EOR", &a::EOR, &a::ABS, 4 },{ "LSR", &a::LSR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BVC", &a::BVC, &a::REL, 2 },{ "EOR", &a::EOR, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ZPX, 4 },{ "LSR", &a::LSR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLI", &a::CLI, &a::IMP, 2 },{ "EOR", &a::EOR, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "EOR", &a::EOR, &a::ABX, 4 },{ "LSR", &a::LSR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "RTS", &a::RTS, &a::IMP, 6 },{ "ADC", &a::ADC, &a::IZX, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 3 },{ "ADC", &a::ADC, &a::ZPO, 3 },{ "ROR", &a::ROR, &a::ZPO, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "PLA", &a::PLA, &a::IMP, 4 },{ "ADC", &a::ADC, &a::IMM, 2 },{ "ROR", &a::ROR, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "JMP", &a::JMP, &a::IND, 5 },{ "ADC", &a::ADC, &a::ABS, 4 },{ "ROR", &a::ROR, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BVS", &a::BVS, &a::REL, 2 },{ "ADC", &a::ADC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ZPX, 4 },{ "ROR", &a::ROR, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SEI", &a::SEI, &a::IMP, 2 },{ "ADC", &a::ADC, &a::ABY, 4 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "ADC", &a::ADC, &a::ABX, 4 },{ "ROR", &a::ROR, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "???", &a::NOP, &a::IMP, 2 },{ "STA", &a::STA, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZPO, 3 },{ "STA", &a::STA, &a::ZPO, 3 },{ "STX", &a::STX, &a::ZPO, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "DEY", &a::DEY, &a::IMP, 2 },{ "???", &a::NOP, &a::IMP, 2 },{ "TXA", &a::TXA, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "STY", &a::STY, &a::ABS, 4 },{ "STA", &a::STA, &a::ABS, 4 },{ "STX", &a::STX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
		{ "BCC", &a::BCC, &a::REL, 2 },{ "STA", &a::STA, &a::IZY, 6 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "STY", &a::STY, &a::ZPX, 4 },{ "STA", &a::STA, &a::ZPX, 4 },{ "STX", &a::STX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "TYA", &a::TYA, &a::IMP, 2 },{ "STA", &a::STA, &a::ABY, 5 },{ "TXS", &a::TXS, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::NOP, &a::IMP, 5 },{ "STA", &a::STA, &a::ABX, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "???", &a::XXX, &a::IMP, 5 },
		{ "LDY", &a::LDY, &a::IMM, 2 },{ "LDA", &a::LDA, &a::IZX, 6 },{ "LDX", &a::LDX, &a::IMM, 2 },{ "???", &a::XXX, &a::IMP, 6 },{ "LDY", &a::LDY, &a::ZPO, 3 },{ "LDA", &a::LDA, &a::ZPO, 3 },{ "LDX", &a::LDX, &a::ZPO, 3 },{ "???", &a::XXX, &a::IMP, 3 },{ "TAY", &a::TAY, &a::IMP, 2 },{ "LDA", &a::LDA, &a::IMM, 2 },{ "TAX", &a::TAX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "LDY", &a::LDY, &a::ABS, 4 },{ "LDA", &a::LDA, &a::ABS, 4 },{ "LDX", &a::LDX, &a::ABS, 4 },{ "???", &a::XXX, &a::IMP, 4 },
		{ "BCS", &a::BCS, &a::REL, 2 },{ "LDA", &a::LDA, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 5 },{ "LDY", &a::LDY, &a::ZPX, 4 },{ "LDA", &a::LDA, &a::ZPX, 4 },{ "LDX", &a::LDX, &a::ZPY, 4 },{ "???", &a::XXX, &a::IMP, 4 },{ "CLV", &a::CLV, &a::IMP, 2 },{ "LDA", &a::LDA, &a::ABY, 4 },{ "TSX", &a::TSX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 4 },{ "LDY", &a::LDY, &a::ABX, 4 },{ "LDA", &a::LDA, &a::ABX, 4 },{ "LDX", &a::LDX, &a::ABY, 4 },{ "???", &a::XXX, &a::IMP, 4 },
		{ "CPY", &a::CPY, &a::IMM, 2 },{ "CMP", &a::CMP, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPY", &a::CPY, &a::ZPO, 3 },{ "CMP", &a::CMP, &a::ZPO, 3 },{ "DEC", &a::DEC, &a::ZPO, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INY", &a::INY, &a::IMP, 2 },{ "CMP", &a::CMP, &a::IMM, 2 },{ "DEX", &a::DEX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 2 },{ "CPY", &a::CPY, &a::ABS, 4 },{ "CMP", &a::CMP, &a::ABS, 4 },{ "DEC", &a::DEC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BNE", &a::BNE, &a::REL, 2 },{ "CMP", &a::CMP, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ZPX, 4 },{ "DEC", &a::DEC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "CLD", &a::CLD, &a::IMP, 2 },{ "CMP", &a::CMP, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "CMP", &a::CMP, &a::ABX, 4 },{ "DEC", &a::DEC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 },
		{ "CPX", &a::CPX, &a::IMM, 2 },{ "SBC", &a::SBC, &a::IZX, 6 },{ "???", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "CPX", &a::CPX, &a::ZPO, 3 },{ "SBC", &a::SBC, &a::ZPO, 3 },{ "INC", &a::INC, &a::ZPO, 5 },{ "???", &a::XXX, &a::IMP, 5 },{ "INX", &a::INX, &a::IMP, 2 },{ "SBC", &a::SBC, &a::IMM, 2 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::SBC, &a::IMP, 2 },{ "CPX", &a::CPX, &a::ABS, 4 },{ "SBC", &a::SBC, &a::ABS, 4 },{ "INC", &a::INC, &a::ABS, 6 },{ "???", &a::XXX, &a::IMP, 6 },
		{ "BEQ", &a::BEQ, &a::REL, 2 },{ "SBC", &a::SBC, &a::IZY, 5 },{ "???", &a::XXX, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 8 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ZPX, 4 },{ "INC", &a::INC, &a::ZPX, 6 },{ "???", &a::XXX, &a::IMP, 6 },{ "SED", &a::SED, &a::IMP, 2 },{ "SBC", &a::SBC, &a::ABY, 4 },{ "NOP", &a::NOP, &a::IMP, 2 },{ "???", &a::XXX, &a::IMP, 7 },{ "???", &a::NOP, &a::IMP, 4 },{ "SBC", &a::SBC, &a::ABX, 4 },{ "INC", &a::INC, &a::ABX, 7 },{ "???", &a::XXX, &a::IMP, 7 }
	};
    

}

olc6502::~olc6502(){


}



uint8_t olc6502::read(uint16_t a){

    return bus->read(a, false); //call bus read function directly

}

void olc6502::write(uint16_t a, uint8_t d){

    return bus->write(a, d);

}

void olc6502::clock(){

	if(cycles == 0){

		opcode = read(pc); //get opcode from program counter. Reading from RAM (on bus)
		pc++;

		cycles = lookup[opcode].cycles; // set required cycles for instruction

		//Following functions return 1 or 0 for additional clock cycles
		uint8_t extra_cycles_1 = (this->*lookup[opcode].addrmode)(); // addresssing mode function
		uint8_t extra_cycles_2 = (this->*lookup[opcode].operate)(); // operation to be performed

		cycles += (extra_cycles_1 + extra_cycles_2);
	}
	cycles--;

}


// ************************ Flag Functions *********************************
uint8_t olc6502::GetFlag(FLAGS6502 f){

	return ((status & f) > 0) ? 1 : 0;

}


void olc6502::SetFlag(FLAGS6502 f, bool v){

	if(v){
		status |= f;
	}else{
		status &= ~f;
	}

}

// *************************************************************************

//***************************** Addressing Modes****************************
uint8_t olc6502::IMP(){

	fetched = a;
	return 0;

}


//Zero page addressing: The byte of data is in page 00
// *Note: The operand is the memory address. 
uint8_t olc6502::ZPO(){

	addr_abs = read(pc);
	pc++; //always increment once after read
	addr_abs &= 0x00FF;
	return 0;

}


// Full address
uint8_t olc6502::ABS(){

	uint16_t lo = read(pc);
	pc++;
	uint16_t hi = read(pc);
	pc++;
	addr_abs = (hi << 8) | lo; // 16 bit address word
	return 0;

}

//Data implied as part of the next instruction.
uint8_t olc6502::IMM(){

	addr_abs = pc++; // Data is the next byte (Program Counter is 16 bits)
	return 0;

}

// Zero Page Addressing with X register offset. Useful for iterating through arrays in memory. Like an array in c++ (ary[x])
uint8_t olc6502::ZPX(){

	addr_abs = (read(pc) + x); //Address supplied has contents of x register added to it.
	pc++;
	addr_abs &= 0x00FF;
	return 0;

}

//Zero Page Addressing with Y register offset. Same as ZPX but with Y register
uint8_t olc6502::ZPY(){

	addr_abs = (read(pc) + y); //Address supplied has contents of y register added to it.
	pc++;
	addr_abs &= 0x00FF;
	return 0;

}


uint8_t olc6502::REL(){


}

// Absolute address with X offset.
// Carry bit from low carries to high after x offset and therefore changed PAGE.
uint8_t olc6502::ABX(){

	uint16_t lo = read(pc);
	pc++;
	uint16_t hi = read(pc);
	pc++;

	addr_abs = (hi << 8) | lo;
	addr_abs += x;

	if( (addr_abs & 0xFF00) != (hi << 8) ){ // If address + x carries over to the high byte, indicate another clock cycle is needed
		return 1;
	}

	return 0;
}


uint8_t olc6502::ABY(){

	uint16_t lo = read(pc);
	pc++;
	uint16_t hi = read(pc);
	pc++;

	addr_abs = (hi << 8) | lo;
	addr_abs += y;

	if( (addr_abs & 0xFF00) != (hi << 8) ){
		return 1;
	}

	return 0;
}

// Indirect addressing below

//OneLoneCoder's words below:
// "
// Address Mode: Indirect
// The supplied 16-bit address is read to get the actual 16-bit address. This is
// instruction is unusual in that it has a bug in the hardware! To emulate its
// function accurately, we also need to emulate this bug. If the low byte of the
// supplied address is 0xFF, then to read the high byte of the actual address
// we need to cross a page boundary. This doesnt actually work on the chip as 
// designed, instead it wraps back around in the same page, yielding an 
// invalid actual address. 
// "

uint8_t olc6502::IND(){

	uint16_t ptr_lo = read(pc);
	pc++;
	uint16_t ptr_hi = read(pc);
	pc++;

	uint16_t ptr = (ptr_hi << 8) | ptr_lo;

	if (ptr_lo == 0x00FF) // Simulate page boundary due to hardware bug
	{
		addr_abs = (read(ptr & 0xFF00) << 8) | read(ptr + 0);
	}
	else // Normal operation
	{
		addr_abs = (read(ptr + 1) << 8) | read(ptr + 0);
	}

	return 0;

}

uint8_t olc6502::IZX(){

	uint16_t t = read(pc);
	pc++;
	uint16_t lo = read((uint16_t)(t + (uint16_t)x) & 0x00FF);
	uint16_t hi = read((uint16_t)(t + (uint16_t)x + 1) & 0x00FF);

	addr_abs = (hi << 8) | lo;
	
	return 0;

}


uint8_t olc6502::IZY(){

	uint16_t t = read(pc);
	pc++;

	uint16_t lo = read(t & 0x00FF);
	uint16_t hi = read((t + 1) & 0x00FF);

	addr_abs = (hi << 8) | lo;
	addr_abs += y;

	if((addr_abs & 0xFF00) != (hi << 8)){
		return 1;
	}else {
		return 0;
	}
}

uint_fast8_t olc6502::REL(){

	addr_rel = read(pc);
	pc++;

	if(addr_rel & 0x80){
		addr_rel |= 0xFF00;
	}
	return 0;
}

//*******************************Instructions************************

uint8_t olc6502::fetch(){

	if (!(lookup[opcode].addrmode == &olc6502::IMP)){
		fetched = read(addr_abs);
	}
	return fetched;
}

// AND operator between acc and fetched
uint8_t olc6502::AND(){

	fetch();
	a &= fetched;

	SetFlag(Z, a == 0x00);

	// if (bit 7) == 1
	SetFlag(N, a & 0x80);

	//add clock cycles, reference datasheet 
	return 1; 
}

//branch instruction
uint8_t olc6502::BCS(){

	// if carry bit is 1
	if(GetFlag(C) == 1){
		cycles++;
		addr_abs = pc + addr_rel;

		//if crosses page boundry then need another clock cycle
		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}


//Branch if carry clear
uint8_t olc6502::BCC(){
	
	if(GetFlag(C) == 0){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Branch if equal
uint8_t olc6502::BEQ(){
	
	if(GetFlag(Z) == 1){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Branch if negative
uint8_t olc6502::BMI(){
	
	if(GetFlag(N) == 1){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Branch if not equal
uint8_t olc6502::BNE(){
	
	if(GetFlag(Z) == 0){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Branch if positive
uint8_t olc6502::BPL(){
	
	if(GetFlag(N) == 0){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Branch if overflow
uint8_t olc6502::BVC(){
	
	if(GetFlag(V) == 0){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Branch if not overflow
uint8_t olc6502::BVS(){
	
	if(GetFlag(V) == 1){
		cycles++;
		addr_abs = pc + addr_rel;

		if(addr_abs & 0xFF00 != (pc & 0xFF00)){
			cycles++;
		}
		pc = addr_abs;
	}
	return 0;
}

//Clear carry bit
uint8_t olc6502::CLC(){

	SetFlag(C, false);
	return 0;
}

