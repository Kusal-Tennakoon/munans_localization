//============================================================================
// Name        : Log.h
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Library for displaying messages on the console
//============================================================================

#ifndef LOG_H_
#define LOG_H_

#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include "quickmath.h"

namespace Log {

/*Class of data logging functions*/
class log {
public:

	/*Prints an message as in information*/
	void info(std::string message);

	/*Prints an message as in warning*/
	void warning(std::string message);

	/*Prints an message as an error*/
	void error(std::string message);

	/*Prints an message as a plain text inline*/
	void print(std::string message);

	/*Prints an message as a plain text and returns to a newline*/
	void println(std::string message);
};

/*Function to convert a given no. into a string
 * 		num - input number
 * 		return = no. as a string
 * */
template<typename T>
std::string toString(T num);

/*Function to print a matrix
 * 		matrix - input matrix
 * */
template<typename T>
void printMat(std::vector<std::vector<T>> matrix);

/*Function to print a vector
 * 		vector - input vector
 * */
template<typename T>
void printVec(std::vector<T> vector);

}
;

template<typename T>
std::string Log::toString(T num) {
	std::stringstream ss;
	ss << num;
	std::string out_string = ss.str();

	return out_string;
}
;

template<typename T>
void Log::printMat(std::vector<std::vector<T>> matrix) {

	std::cout << "[ ";
	for (int i = 0; i <= matrix.size() - 1; i++) {
		std::cout << "   ";
		for (int j = 0; j <= matrix.at(0).size() - 1; j++) {

			std::cout << matrix.at(i).at(j) << "      ";
		}
		;
		if (i < matrix.size() - 1) {
			std::cout << std::endl;
		} else {
			std::cout << "";
		};
	}
	;
	std::cout << "]" << std::endl;
}
;

template<typename T>
void Log::printVec(std::vector<T> vector) {

	std::cout << "[ ";
	for (int i = 0; i <= vector.size() - 1; i++) {
		std::cout << vector.at(i) << "  ";
	}
	;
	std::cout << "]" << std::endl;
}
;

#endif /* LOG_H_ */
