#ifndef COMMON_H_
#define COMMON_H_

#include<string>
#include<iostream>
#include<exception>

void rawPrint(const std::string& input);

template<typename T>
void contentPrint(const std::string& print_head, const T& input){
    try{
        std::cout << "[" << print_head << "]:" << input << std::endl;
    }
    catch(std::exception& ex){
        std::cout << "Print error: " << ex.what() << std::endl;
    }
}

#endif
