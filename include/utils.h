

template <typename T>
void Convert7ParameterQuaternionRepresentationIntoMatrix(const T* X, T* XM);

template <typename T>
T bilinearWithoutDepth(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& intensity, const T& x, const T& y);

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> comp_all_intensities(const Eigen::Matrix<T, 2, Eigen::Dynamic> & pixels_img2 , const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& img2_intensity );

template <typename T>
bool check_pixels_in_img(const Eigen::Matrix<T, 2, Eigen::Dynamic>& pixels_img2, int width, int height);

