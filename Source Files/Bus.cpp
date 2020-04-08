#include "Bus.h"

Bus::Bus(){
    //clear RAM
    for (auto &i : ram) i = 0x00;

    //Connect CPU
    cpu.ConnectBus(this);
}

Bus::~Bus(){

}


void Bus::write(uint16_t addr, uint8_t data){
    //Making sure address is in range of machine
    if(addr >= 0x0000 & addr <= 0xFFFF){
        ram[addr] = data;
    }

}
        

uint8_t Bus::read(uint16_t addr, bool bReadOnly = false){

    //Making sure address is in range of machine
    if(addr >= 0x0000 & addr <= 0xFFFF){
        return ram[addr];
    }

    return 0x00;
}