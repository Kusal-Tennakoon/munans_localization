//=====================================================================================
// Name        : quickmath.h
// Author      : Kusal Tennakoon
// Version     :
// Copyright   : MUN/ISL/kbtennakoon
// Description : Custom tempalate Vector and Matrix library with built in functions for 
//				 computing mean, max ,std.dev, row sum, col sum, and printing.
//=====================================================================================

#ifndef QUICKMATH_H_
#define QUICKMATH_H_

#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <array>
#include <vector>
#include <algorithm>
#include <numeric>
#include <sstream>

namespace Qmath {

/*Data structures*/
/*////////////////////*/

/* Data structure to store the size of a matrix
 * 		no_of_rows - no of no_of_rows
 *  	no_of_cols - no of columns
 */
struct Size {
	int rows = 0;
	int cols = 0;
};

/* Data structure to store max value and ID
 * 		value - maximum value
 *  	ID - position of the value
 */
template<typename T>
struct Max {
	T value = 0;
	int ID = 0;
};

/*Function declarations*/
/*////////////////////*/

/*Matrix Class*/
template<typename T>
class Matrix {
private:
	std::vector<std::vector<T>> matrix;
	int no_of_rows;
	int no_of_cols;
public:
	/*Default constructor*/
	Matrix();

	/*Constructor*/
	Matrix(int n_rows = 1, int n_cols = 1);

	/*Constructor for a given input matrix*/
	Matrix(std::vector<std::vector<T>> mat);

	/*Function to obtain the no. of rows of the matrix
	 * 		returns : no. of rows
	 */
	int rows();

	/*Function to obtain the no. of columns of the matrix
	 * 		returns : no. of columns
	 */
	int cols();

	/*Function to assign values to the matrix
	 * 		i - row
	 * 		j - column
	 * 		value - value to be assigned
	 */
	void Set(int i, int j, T value);

	/*Function to retrieve values from the matrix
	 * 		i - row
	 * 		j - column
	 * 		return : value at (i,j)
	 */
	T Get(int i, int j);

	/*Function to compute the sum of the matrix
	 * 		matrix - input matrix
	 * 		return : Size struct
	 */
	T sum();

	/*Function to compute the row-wise sum of the matrix
	 * 		Rsum - row sum container
	 */
	void rowSum(std::vector<T> &RSum);

	/*Function to compute the column-wise sum of the matrix
	 * 		RCum - column sum container
	 */
	void colSum(std::vector<T> &CSum);

	/*Function to compute a given no of maximum values of the matrix
	 * 		maximums - maximum values container
	 */
	void max(int num, std::vector<Max<T>> &maximums);

	/*Function to compute the mean of the matrix
	 * 		return : mean
	 */
	double mean();

	/*Function to compute the standard deviation of the matrix
	 * 		return : standard deviation
	 */
	double stdDev();

	/*Function to print the matrix */
	void print();
};

/*Vector Class*/
template<typename T>
class Vector {
private:
	std::vector<T> vector;
	int length;
public:
	/*Default constructor*/
	Vector();

	/*Constructor for a given input vector*/
	Vector(std::vector<T> v);

	/*Constructor for a given no. of elements*/
	Vector(int no_of_elements = 1);

	/*Function to obtain the no.of elements in the matrix
	 * 		returns : no. of elements
	 */
	int size();

	/*Function to assign values to the vector
	 * 		i - position
	 */
	void Set(int i, T value);

	/*Function to retrieve values from the vector
	 * 		i - row
	 */
	T Get(int i);

	/*Function to compute the sum of the vector
	 * 		return : sum
	 */
	T sum();

	/*Function to compute a given no of maximum values ofthe vector
	 * 		return : vector of Max structs
	 */
	void max(int num, std::vector<Max<T>> &maximums);

	/*Function to compute the mean of the vector
	 * 		return : mean
	 */
	double mean();

	/*Function to compute the standard deviation of the vector
	 * 		return : standard deviation
	 */
	double stdDev();

	/*Function to print the vector */
	void print();

};
}
;

/*Function definitions*/
/*////////////////////*/

template<typename T>
Qmath::Matrix<T>::Matrix() {

}
;

template<typename T>
Qmath::Matrix<T>::Matrix(std::vector<std::vector<T>> mat) {

	for (int i = 0; i <= mat.size() - 1; i++) {
		matrix.push_back(mat.at(i));
	}
	;
	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();
}
;

template<typename T>
Qmath::Matrix<T>::Matrix(int n_rows, int n_cols) {

	std::vector<T> Row;
	Row.assign(n_cols, 0);
	matrix.assign(n_rows, Row);

	no_of_rows = n_rows;
	no_of_cols = n_cols;
}
;

template<typename T>
int Qmath::Matrix<T>::rows() {
	no_of_rows = matrix.size();

	return no_of_rows;
}
;

template<typename T>
int Qmath::Matrix<T>::cols() {
	no_of_cols = matrix.at(0).size();

	return no_of_cols;
}
;

template<typename T>
void Qmath::Matrix<T>::Set(int row, int col, T value) {

	matrix.at(row).at(col) = value;
}
;

template<typename T>
T Qmath::Matrix<T>::Get(int row, int col) {

	T retval = matrix.at(row).at(col);

	return retval;
}
;

template<typename T>
void Qmath::Matrix<T>::colSum(std::vector<T> &CSum) {

	T sum = 0;
	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();

	//Calculating the column sum
	for (int i = 0; i <= no_of_cols - 1; i++) {
		for (int j = 0; j <= no_of_rows - 1; j++) {
			sum += matrix.at(j).at(i);
		}
		;
		// Appending the value to CSum
		CSum.push_back(sum);
		sum = 0;
	};
}
;

template<typename T>
void Qmath::Matrix<T>::rowSum(std::vector<T> &RSum) {

	T sum = 0;
	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();

	//	Calculating the sum
	for (int i = 0; i <= no_of_rows - 1; i++) {
		for (int j = 0; j <= no_of_cols - 1; j++) {
			sum += matrix.at(i).at(j);
		}
		;
		// Appending the value to RSum
		RSum.push_back(sum);
		sum = 0;
	};
}
;

template<typename T>
T Qmath::Matrix<T>::sum() {

	T sum = 0;
	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();

	//	Calculating the sum
	for (int i = 0; i <= no_of_rows - 1; i++) {
		for (int j = 0; j <= no_of_cols - 1; j++) {
			sum += matrix.at(i).at(j);
		};
	}
	;
	return sum;
}
;

template<typename T>
void Qmath::Matrix<T>::max(int num, std::vector<Qmath::Max<T>> &maximums) {

	Qmath::Max<T> max;

	T nil = 0;
	int count = 1;
	int max_m = 0;
	int max_n = 0;

	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();

	// Creating a copy of the matrix in order to preserve the original matrix during replacement of '0'
	std::vector<std::vector<T>> matrix_cpy;

	for (int i = 0; i <= no_of_rows - 1; i++) {
		matrix_cpy.push_back(matrix.at(i));
	}
	;

	// Calculating the maximum
	while (count <= num) {
		for (int i = 0; i <= no_of_rows - 1; i++) {
			for (int j = 0; j <= no_of_cols - 1; j++) {
				if (matrix_cpy.at(i).at(j) > max.value) {
					max.value = matrix_cpy.at(i).at(j);
					max.ID = (no_of_cols) * (i) + (j);
					max_m = i;
					max_n = j;
				};
			};
		}
		;
		//Replacing the current max in the matrix_cpy with 0
		matrix_cpy.at(max_m).at(max_n) = nil;

		maximums.push_back(max);
		max.value = 0;
		count++;
	};
}
;

template<typename T>
double Qmath::Matrix<T>::mean() {

	double mean = 0;
	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();

	//	Calculating the sum
	for (int i = 0; i <= no_of_rows - 1; i++) {
		for (int j = 0; j <= no_of_cols - 1; j++) {
			mean += matrix.at(i).at(j);
		};
	}
	;
	// mean = sum / no. of elements
	mean = (mean / (no_of_rows * no_of_cols));

	return mean;
}
;

template<typename T>
double Qmath::Matrix<T>::stdDev() {

	double mu = 0;
	double sigma = 0;
	no_of_rows = matrix.size();
	no_of_cols = matrix.at(0).size();

	//	Calculating the sum
	for (int i = 0; i <= no_of_rows - 1; i++) {
		for (int j = 0; j <= no_of_cols - 1; j++) {
			mu += matrix.at(i).at(j);
		};
	}
	;
	mu = (mu / (no_of_rows * no_of_cols));

	//	Calculating the standard deviation
	for (int i = 0; i <= no_of_rows - 1; i++) {
		for (int j = 0; j <= no_of_cols - 1; j++) {
			sigma += (matrix.at(i).at(j) - mu) * (matrix.at(i).at(j) - mu);
		};
	}
	;

	sigma = std::sqrt(sigma / (no_of_rows * no_of_cols));

	return sigma;
}
;

template<typename T>
void Qmath::Matrix<T>::print() {

	if (matrix.size() != 0) {

		std::cout << "[  ";
		for (int i = 0; i <= no_of_rows - 1; i++) {
			std::cout << "   ";
			for (int j = 0; j <= no_of_cols - 1; j++) {

				std::cout << matrix.at(i).at(j) << "      ";
			}
			;
			if (i < no_of_rows - 1) {
				std::cout << std::endl;
			} else {
				std::cout << "";
			};
		}
		;
		std::cout << "]" << std::endl;
	} else {
		std::cout << "[]" << std::endl;
	};
}

template<typename T>
Qmath::Vector<T>::Vector() {

}
;

template<typename T>
Qmath::Vector<T>::Vector(std::vector<T> v) :
		vector(v) {

}
;

template<typename T>
Qmath::Vector<T>::Vector(int no_of_elements) {

	vector.assign(no_of_elements, 0);
	length = no_of_elements;
}
;

template<typename T>
int Qmath::Vector<T>::size() {

	length = vector.size();

	return length;
}
;

template<typename T>
void Qmath::Vector<T>::Set(int row, T value) {

	vector.at(row) = value;
}
;

template<typename T>
T Qmath::Vector<T>::Get(int row) {

	T retval = vector.at(row);

	return retval;
}
;

template<typename T>
T Qmath::Vector<T>::sum() {

	T sum = 0;

	//	Calculating the sum
	for (int i = 0; i <= vector.size() - 1; i++) {
		sum += vector.at(i);
	}
	;
	return sum;
}
;

template<typename T>
void Qmath::Vector<T>::max(int num, std::vector<Qmath::Max<T>> &maximums) {

	Qmath::Max<T> max;
	// Creating a copy of the vector in order to preserve the original matrix during replacement of '0'
	std::vector<T> vector_cpy(vector);

	T nil = 0;
	int count = 1;

	// Calculating the maximum
	while (count <= num) {
		for (int i = 0; i <= vector.size() - 1; i++) {
			if (vector_cpy.at(i) > max.value) {
				max.value = vector_cpy.at(i);
				max.ID = i;
			};
		}
		;
		//Replacing the current max in the vector copy with 0
		vector_cpy.at(max.ID) = nil;
		maximums.push_back(max);
		max.value = 0;
		count++;
	};
}
;

template<typename T>
double Qmath::Vector<T>::mean() {
	double sum = 0;
	double mean = 0;

	//	Calculating the sum
	for (int i = 0; i <= vector.size() - 1; i++) {
		sum += vector.at(i);
	}
	;
	// mean = sum / no. of elements
	mean = (sum / (vector.size()));

	return mean;
}
;

template<typename T>
double Qmath::Vector<T>::stdDev() {

	double sigma = 0;
	double mu = 0;

	//	Calculating the sum
	for (int i = 0; i <= vector.size() - 1; i++) {
		mu += vector.at(i);
	}
	;

	// mean = sum / no. of elements
	mu = (mu / (vector.size()));

	//	Calculating the standard deviation
	for (int i = 0; i <= vector.size() - 1; i++) {
		sigma += (vector.at(i) - mu) * (vector.at(i) - mu);
	}
	;

	sigma = std::sqrt(sigma / vector.size());

	return sigma;

}
;

template<typename T>
void Qmath::Vector<T>::print() {

	if (vector.size() != 0) {
		std::cout << "[  ";
		for (int i = 0; i <= vector.size() - 1; i++) {
			std::cout << vector.at(i) << "  ";
		}
		;
		std::cout << "]" << std::endl;
	} else {
		std::cout << "[]" << std::endl;
	};
}
;

#endif /* QUICKMATH_H_ */
