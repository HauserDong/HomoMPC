#include "openGJK.h"
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <vector>



namespace py = pybind11;

PYBIND11_MODULE(opengjkc, m) {
  m.def("gjk",
        // 这里是一个典型的lambda函数
        [](Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor> &arr1,
           Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor> &arr2
           )-> double {

            gkSimplex s;
            gkPolytope bd1;
            gkPolytope bd2;
            bd1.numpoints = arr1.rows();
            std::vector<double *> arr1_rows(arr1.rows());
            for (int i = 0; i < arr1.rows(); ++i)
            arr1_rows[i] = arr1.row(i).data();
            bd1.coord = arr1_rows.data();

            bd2.numpoints = arr2.rows();
            std::vector<double *> arr2_rows(arr2.rows());
            for (int i = 0; i < arr2.rows(); ++i)
            arr2_rows[i] = arr2.row(i).data();
            bd2.coord = arr2_rows.data();

            double dis = 0.0;
            double dir[3];
            
            GJK(bd1, bd2, &s,&dis,dir);

            return dis;
    });

    m.def("gjkDir",
        // 这里是一个典型的lambda函数
        [](Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor> &arr1,
           Eigen::Array<double, Eigen::Dynamic, 3, Eigen::RowMajor> &arr2
           ) -> std::vector<double> {

            gkSimplex s;
            gkPolytope bd1;
            gkPolytope bd2;
            bd1.numpoints = arr1.rows();
            std::vector<double *> arr1_rows(arr1.rows());
            for (int i = 0; i < arr1.rows(); ++i)
            arr1_rows[i] = arr1.row(i).data();
            bd1.coord = arr1_rows.data();

            bd2.numpoints = arr2.rows();
            std::vector<double *> arr2_rows(arr2.rows());
            for (int i = 0; i < arr2.rows(); ++i)
            arr2_rows[i] = arr2.row(i).data();
            bd2.coord = arr2_rows.data();

            double dis = 0.0;
            double dir[3];
            
            GJK(bd1, bd2, &s,&dis,dir);

            std::vector<double> Dir = {0.0,0.0,0.0};
            Dir[0] = dir[0];
            Dir[1] = dir[1];
            Dir[2] = dir[2];
            return Dir;
            
    });
}