//============================================================================
// Name        : Log.cpp
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Library for displaying messages on the console
//============================================================================

#include <iostream>
#include <string>
#include "Log.h"

/*Color definitions according to ANSI standard*/
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

void Log::log::info(std::string message) {

	std::cout << CYAN << "[INFO] : " << message << RESET << std::endl
			<< std::endl;
}
;

void Log::log::warning(std::string message) {

	std::cout << YELLOW << "[WARNING] : " << message << RESET << std::endl
			<< std::endl;
}
;

void Log::log::error(std::string message) {

	std::cout << RED << "[ERROR] : " << message << RESET << std::endl
			<< std::endl;
}
;

void Log::log::print(std::string message) {

	std::cout << message << RESET;
}
;

void Log::log::println(std::string message) {

	std::cout << message << RESET << std::endl << std::endl;
}
;
