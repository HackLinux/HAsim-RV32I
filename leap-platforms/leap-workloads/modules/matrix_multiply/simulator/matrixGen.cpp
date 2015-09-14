#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <complex>
#include <iomanip>
#include <bitset>
using namespace std;

unsigned int dataByteWidth  = 4;

complex<double> complexMultiply(complex<double> x1, complex<double> x2)
{
    int x1_real = int(x1.real());
    int x2_real = int(x2.real());
    int x1_imag = int(x1.imag());
    int x2_imag = int(x2.imag());
    int r1 = int(bitset<16>(x1_real*x2_real).to_ulong());
    int r2 = int(bitset<16>(x1_imag*x2_imag).to_ulong());
    int r3 = int(bitset<16>(x1_real*x2_imag).to_ulong());
    int r4 = int(bitset<16>(x1_imag*x2_real).to_ulong());
    int m_real = int(bitset<16>(r1-r2).to_ulong());
    int m_imag = int(bitset<16>(r3+r4).to_ulong());
    return complex<double>(double(m_real), double(m_imag));
}

complex<double> complexAdd(complex<double> x1, complex<double> x2)
{
    int x1_real = int(x1.real());
    int x2_real = int(x2.real());
    int x1_imag = int(x1.imag());
    int x2_imag = int(x2.imag());
    int r1 = int(bitset<16>(x1_real+x2_real).to_ulong());
    int r2 = int(bitset<16>(x1_imag+x2_imag).to_ulong());
    return complex<double>(double(r1), double(r2));
}

int main ( int argc, char *argv[] )
{
    if ( argc < 4 )
        cout<<"usage: "<< argv[0] <<" <matrix A x size> <matrix A y size> <matrix B x size> [genGoldenAnswer]"<<endl;
    else
    {
        unsigned int matrixSizeAx = atoi(argv[1]);
        unsigned int matrixSizeAy = atoi(argv[2]);
        unsigned int matrixSizeBx = atoi(argv[3]);
        unsigned int matrixSizeBy = matrixSizeAx;
        unsigned int matrixSizeCx = matrixSizeBx;
        unsigned int matrixSizeCy = matrixSizeAy;
        bool genGoldenAnswer = false;
        cout<<"Matrix multiply: Matrix A: "<<matrixSizeAx<<"x"<<matrixSizeAy;
        cout<<" Matrix B: "<<matrixSizeBx<<"x"<<matrixSizeBy<<" Matrix C: "<<matrixSizeCx<<"x"<<matrixSizeCy<<endl;
       
        if (argc == 5 && string(argv[4]) == string("genGoldenAnswer"))
        {
            genGoldenAnswer = true; 
            cout<<"generate golden matrix C"<<endl;
        }

        stringstream fileBaseName;
        fileBaseName << "../benchmark/matrix_multiply_"<<matrixSizeAx<<"x"<<matrixSizeAy<<"_"<<matrixSizeBx<<"x"<<matrixSizeBy<<"_";

        
        // generate matrices
        complex<double>** matrixA; 
        complex<double>** matrixB;

        if (genGoldenAnswer)
        {
            matrixA = new complex<double>* [matrixSizeAy];
            for (unsigned int i = 0; i < matrixSizeAy; i++)
                matrixA[i] = new complex<double> [matrixSizeAx];
            matrixB = new complex<double>* [matrixSizeBy];
            for (unsigned int i = 0; i < matrixSizeBy; i++)
                matrixB[i] = new complex<double> [matrixSizeBx];
        }

        ofstream outfile;
        ofstream outfileHex;
        outfile.open((fileBaseName.str()+string("matrixA.dat")).c_str(), ios::out | ios::binary);
        outfileHex.open((fileBaseName.str()+string("matrixA.hex")).c_str(), ios::out);
        for (unsigned int y = 0; y < matrixSizeAy; y++)
        {
            for (unsigned int x = 0; x < matrixSizeAx; x++)
            {
                int real = rand() % 65536;
                int imag = rand() % 65536;
                outfile.write((char*)&imag, dataByteWidth/2);
                outfile.write((char*)&real, dataByteWidth/2);
                outfileHex << hex << setfill('0') << setw(2) << real << imag <<" ";
                if (genGoldenAnswer)
                    matrixA[y][x] = complex<double>(double(real), double(imag));
            }
            outfileHex << endl;
        }
        outfile.close();
        outfileHex.close();
        
        outfile.open((fileBaseName.str()+string("matrixB.dat")).c_str(), ios::out | ios::binary);
        outfileHex.open((fileBaseName.str()+string("matrixB.hex")).c_str(), ios::out);
        for (unsigned int y = 0; y < matrixSizeBy; y++)
        {
            for (unsigned int x = 0; x < matrixSizeBx; x++)
            {
                int real = rand() % 65536;
                int imag = rand() % 65536;
                outfile.write((char*)&imag, dataByteWidth/2);
                outfile.write((char*)&real, dataByteWidth/2);
                outfileHex << hex << setfill('0') << setw(2) << real << imag <<" ";
                if (genGoldenAnswer)
                    matrixB[y][x] = complex<double>(double(real), double(imag));
            }
            outfileHex << endl;
        }
        outfile.close();
        outfileHex.close();
    
        if (genGoldenAnswer)
        {
            outfile.open((fileBaseName.str()+string("matrixC.dat")).c_str(), ios::out | ios::binary);
            outfileHex.open((fileBaseName.str()+string("matrixC.hex")).c_str(), ios::out);
            for (unsigned int cy = 0; cy < matrixSizeCy; cy++)
            {
                for (unsigned int cx = 0; cx < matrixSizeCx; cx++)
                {
                    complex<double> tmp(0.0, 0.0);
                    for (unsigned int i = 0; i < matrixSizeAx; i++)
                    {
                        //tmp += matrixA[cy][i] * matrixB[i][cx];
                        tmp = complexAdd(complexMultiply(matrixA[cy][i], matrixB[i][cx]), tmp);
                        // if (cx == 0 && cy == 0)
                        // {
                        //     //complex<double> m = matrixA[cy][i] * matrixB[i][cx];
                        //     complex<double> m = complexMultiply(matrixA[cy][i], matrixB[i][cx]);
                        //     cout<<"matrixA[0]["<<i<<"]="<<matrixA[cy][i].real()<<"+"<<matrixA[cy][i].imag()<<"i (";
                        //     cout<< hex <<setfill('0')<<setw(2)<<int(matrixA[cy][i].real())<<int(matrixA[cy][i].imag())<<"), ";
                        //     cout<<"matrixB["<<i<<"][0]="<<matrixB[i][cx].real()<<"+"<<matrixB[i][cx].imag()<<"i (";
                        //     cout<< hex <<setfill('0')<<setw(2)<<int(matrixB[i][cx].real())<<int(matrixB[i][cx].imag())<<"), ";
                        //     cout<<"multiply result="<<m.real()<<"+"<<m.imag()<<"i (";
                        //     cout<< hex <<setfill('0')<<setw(2)<<int(m.real())<<int(m.imag())<<"), ";
                        //     cout<<"tmp= "<<tmp.real()<<"+"<<tmp.imag()<<"i (";
                        //     cout<< hex <<setfill('0')<<setw(2)<<int(tmp.real())<<int(tmp.imag())<<")"<<endl;
                        // }
                    }
                    int real = int(tmp.real());
                    int imag = int(tmp.imag());
                    outfile.write((char*)&imag, dataByteWidth/2);
                    outfile.write((char*)&real, dataByteWidth/2);
                    outfileHex << hex << setfill('0') << setw(2) << real << imag <<" ";
                }
                outfileHex << endl;
            }
            outfile.close();
            outfileHex.close();
            
            for (unsigned int i = 0; i < matrixSizeAy; i++)
                delete [] matrixA[i];
            for (unsigned int i = 0; i < matrixSizeBy; i++)
                delete [] matrixB[i];
            delete [] matrixA;
            delete [] matrixB;
        }
    }
    
    return 0;
}

