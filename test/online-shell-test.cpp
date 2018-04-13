/*
 * online-shell-test.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: arasdar
 */
// Example program
#include <iostream>
#include <string>

double func(const int, const float);
// double func(const int&, const float&);
double func2(const int&, const float&);
double func(const int*, const float*);
double func2(const int*, const float*);

int main()
{
//   std::string name;
//   std::cout << "What is your name? ";
//   getline (std::cin, name);
//   std::cout << "Hello, " << name << "!\n";

// // Test
// std::cout << func(1, 2) << std::endl;
// std::cout << func2(1, 2) << std::endl;

// //Test2
// int x[3]={0, 1, 2};
// float y[3]={0, 1, 2};
// std::cout << func(x, y) << std::endl;
// std::cout << func2(x, y) << std::endl;

// // Test3
// // delete[] z;
// char z[3]={'s', 'w', 'e'};
// // char z[3]={s, w, e}; // ERROR
// // int* z=new int [5]
// // z={1, 2, 3, 4, 5};
// // std::cout << z << " " << *z << " " << z+2 << " " << *(z+2); // << x[2] << endl;
// std::cout << *(z+2) << std::endl;
// std::cout << z[1] << std::endl;

// // Test4
// char* a(new char[4]);
// *a='0';
// *(a+1)='1';
// a[2]='2';
// a[3]='3';
// std::cout << *a << a[1] << *(a+2) << a[3] << a << std::endl;
// std::cout << a << std::endl;

// Test 5
// int a[0]={}; // zero blocks and only address
// int& a;
int b[1]={1}; // only one block/classtype/checkibg the class type/ emphasizing on the classtype
float c[2]={2, 3}; // This is an actuall buffer or array or memory buffer

// std::cout << a << " " << *a << std::endl;
std::cout << b << " "<< *b << std::endl;
std::cout << c << " " << *c << " " << c[1] << std::endl;

// Test
std::cout << func2(b[0], c[1]) << std::endl;
std::cout << func2(b, c) << std::endl;

return 0;
}

// double func(const int x, const float y)
// {
//     return x*y;
// }

// double func(const int& x, const float& y)
// {
//     return x*y;
// }

double func2(const int& x, const float& y)
{
    return x*y;
}

// double func(const int* x, const float* y)
// {
//     return x[1]*y[2];
// }

double func2(const int* x, const float* y)
{
    return x[0]*y[1];
}



//////////////////////////////////////////////////////

/*
 * online-shell-test.cpp
 *
 *  Created on: Apr 10, 2018
 *      Author: arasdar
 */
// Example program
#include <iostream>
#include <string>

// double func(const int, const float);
// double func(const int&, const float&);
// double func(const int&, const float&);
// double func(const int*, const float*);
// double func(const int*, const float*);

int main()
{

// // Test
// int n=1;
// int a[n];

// Test 5
int x=1;
int a[0]={}; // zero blocks and only address
int b[1]={1}; // only one block/classtype/checkibg the class type/ emphasizing on the classtype
float c[2]={2, 3}; // This is an actuall buffer or array or memory buffer

std::cout << &x << " " << x << std::endl;
std::cout << a << " " << *a << std::endl;
std::cout << b << " "<< *b << std::endl;
std::cout << c << " " << *c << " " << c[1] << std::endl;

// // Test
// std::cout << func2(b[0], c[1]) << std::endl;
// std::cout << func2(b, c) << std::endl;

return 0;
}

// double func(const int x, const float y)
// {
//     return x*y;
// }

// double func(const int& x, const float& y)
// {
//     return x*y;
// }

// double func(const int& x, const float& y)
// {
//     return x*y;
// }

// double func(const int* x, const float* y)
// {
//     return x[1]*y[2];
// }

// double func(const int* x, const float* y)
// {
//     return x[0]*y[1];
// }

