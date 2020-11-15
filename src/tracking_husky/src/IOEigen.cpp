#include "IOEigen.h"
using namespace IOeigen;

bool IOEigen::readMatrix(string filename, Eigen::MatrixXd &M){
    ifstream fin(filename.c_str());
    if(!fin.good()){
            return false;
    }
    int rows, cols;
    //read header matrix Name rows cols
    fin >> rows; fin >> cols;
    M.resize(rows,cols);
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            fin >> M(i,j);
        }
    }

    return true;
}

void IOEigen::writeMatrix(string filename, Eigen::MatrixXd const M, bool header){
    string str_header = "";
    ofstream myfile;
    myfile.open (filename.c_str());
    if(header){
        str_header + to_string(M.rows()) + " " + to_string(M.cols()) + "\n";
        myfile << header;
    }
    myfile << M;
    myfile.close();
}