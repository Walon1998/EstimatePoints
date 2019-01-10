#include <iostream>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

using namespace Eigen;
using namespace std;

//!
//! \brief buildDistanceLSQMatrix Efficiently build the system matrix
//! mapping positions to distances.
//! \param n Number of points (including $x_1$).
//! \return The system matrix $A$.
//!

SparseMatrix<double> buildDistanceLSQMatrix(int n) {
    SparseMatrix<double> A(n * (n - 1) / 2, n - 1);
    vector<Triplet<double>> triplets;
    int length = n * (n - 1) / 2;

    int col = 0;
    int counter = 0;
//    triplets.reserve(n * (n - 1));
    for (int j = 0; j < n - 1; ++j) {
        triplets.push_back(Triplet<double>(j, j, 1));
    }

    for (int i = n - 1; i < length; ++i) {
        counter++;
        if (counter > n - 2) {
            counter = col + 2;
            col++;
        }
        triplets.push_back(Triplet<double>(i, col, -1));
        triplets.push_back(Triplet<double>(i, counter, 1));

    }


    A.setFromTriplets(triplets.begin(), triplets.end());
    A.makeCompressed();


    return A;
}


//!
//! \brief estimatePointsPositions Return positions (without $x_1$).
//! The coordinate $x_1$ is assumed $x_1 = 0$.
//! \param D An $n \times n$ anti-symmetric matrix of distances.
//! \return Vector of positions $x_2, \dots, x_n$.
//!

VectorXd estimatePointsPositions(const MatrixXd &D) {
    int size = D.cols();
//    int size = 3;
    MatrixXd A = buildDistanceLSQMatrix(size);
    MatrixXd AtA = A.transpose() * A;

//    VectorXd b(size * (size - 1));
    VectorXd b(size * 2);
//    cout << size * (size - 1);
    int counter = 0;

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (i < j) {
                b(counter) = -D(i, j);
                counter++;
            }


        }
    }


    MatrixXd Atb = A.transpose() * b;

    cout << Atb << endl;
    cout << AtA << endl;


    VectorXd x = AtA.colPivHouseholderQr().solve(Atb);









    return x;
}


int main() {

    int n = 5;

    // PART 1: build and print system matrix A
    std::cout << "**** PART 1 ****" << std::endl;
    std::cout << "The Matrix A is:"
              << std::endl
              << buildDistanceLSQMatrix(n)
              << std::endl;

    // PART 2: solve the LSQ system and find positions
    std::cout << "**** PART 2 ****" << std::endl;
    // Vector of positions
    n = 5;

    // Build D
    MatrixXd D(n, n);
    D << 0, -2.1, -3, -4.2, -5,
            2.1, 0, -0.9, -2.2, -3.3,
            3, 0.9, 0, -1.3, -1.1,
            4.2, 2.2, 1.3, 0, -1.1,
            5, 3.3, 1.1, 1.1, 0;
    std::cout << "The matrix D is:"
              << std::endl
              << D
              << std::endl;

    // Find out positions
    VectorXd x_recovered = estimatePointsPositions(D);
    std::cout << "The positions [x_2, ..., x_n] obtained from the LSQ system are:"
              << std::endl
              << x_recovered
              << std::endl;
}
