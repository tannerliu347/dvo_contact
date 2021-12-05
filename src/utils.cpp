template <typename T>
void MatrixMultiply(const T* X, const T* Y, T* M, int row0, int col0, int row1, int col1){

//	cout << "Args in Matrix Multiply: " << endl;
//	cout << row0 << ", " << col0 << ", " << row1 << ", " << col1 << endl;
//
//	cout << "First matrix " << endl;
//	PrintMatrix(X, row0, col0);
//
//	cout << "Second matrix " << endl;
//	PrintMatrix(Y, row1, col1);


	// not assuming square matrices
	if (col0 != row1){
		cout << "Wrong args sent to Matrix Multiply: " << endl;
		cout << row0 << ", " << col0 << ", " << row1 << ", " << col1 << endl;
	}

	int xi, yi;
	T r;
	for (int i = 0; i < row0; i++){
		for (int j = 0; j < col1; j++){
			// dot product the ith row of X by the jth column of Y, results in the
			//cout << "i, j " << i << ", " << j << endl;

			r= T(0);
			for (int index = 0; index < col0; index++){
				xi = i*col0 + index; // walk across the row
				yi = index*col1 + j; // walk down the columm

				r += X[xi]*Y[yi];

				//cout << "Mult " << xi << " from first by " << yi << " from second" << endl;
			}
			//cout << "Result stored in " << i*col1 + j << endl;
			//char ch; cin >> ch;
			M[i*col1 + j] = r;
		}
	}
}