/*
  The resulting means of the distributions are 416.64 and 342.48,
  which will be used to offset the distribution.
*/

#include <utility>
#include <iostream>
#include <random>

#define X_NOISE_MEAN_1 416.8
#define X_NOISE_STD_1 0.1

#define X_NOISE_MEAN_2 416.45
#define X_NOISE_STD_2 0.05

#define X_NOISE_MEAN_3 416.25
#define X_NOISE_STD_3 0.05

#define Y_NOISE_MEAN_1 342.7
#define Y_NOISE_STD_1 0.1

#define Y_NOISE_MEAN_2 342.35
#define Y_NOISE_STD_2 0.05

#define Y_NOISE_MEAN_3 341.7
#define Y_NOISE_STD_3 0.2

#define X_MEAN 416.64
#define Y_MEAN 342.48

std::pair<double, double> xybidist(void)
{
  static std::default_random_engine generator;
  static std::uniform_int_distribution<int> uni(0,5);
  static std::normal_distribution<double> x_normal_1(X_NOISE_MEAN_1, X_NOISE_STD_1);
  static std::normal_distribution<double> x_normal_2(X_NOISE_MEAN_2, X_NOISE_STD_2);
  static std::normal_distribution<double> x_normal_3(X_NOISE_MEAN_3, X_NOISE_STD_3);
  static std::normal_distribution<double> y_normal_1(Y_NOISE_MEAN_1, Y_NOISE_STD_1);
  static std::normal_distribution<double> y_normal_2(Y_NOISE_MEAN_2, Y_NOISE_STD_2);
  static std::normal_distribution<double> y_normal_3(Y_NOISE_MEAN_3, Y_NOISE_STD_3);
  int i;
  double x, y;

  i = uni(generator);
  switch (i) {
  case 4:
    x = x_normal_2(generator);
    break;
  case 5:
    x = x_normal_3(generator);
    break;
  default:			// 0, 1, 2, 3
    x = x_normal_1(generator);
    break;
  }

  i = uni(generator);
  switch (i) {
  case 4:
    y = y_normal_2(generator);
    break;
  case 5:
    y = y_normal_3(generator);
    break;
  default:			// 0, 1, 2, 3
    y = y_normal_1(generator);
    break;
  }

  return std::pair<double, double>(x, y);
}

#ifdef __MAIN__

#define NUM 1487

int main(int argc, char *argv[])
{
  for (int t = 0; t < NUM; t++) {
    double x, y;
    std::pair<double, double> p;

    p = xybidist();
    x = p.first - X_MEAN, y = p.second - Y_MEAN;
    std::cout << x << '\t' << y << std::endl;
  }

  return 0;
}

#endif	// __MAIN__
