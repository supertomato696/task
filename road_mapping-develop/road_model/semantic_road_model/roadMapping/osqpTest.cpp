#include <osqp.h>
#include <iostream>
#include <Eigen/Core>


int main()
{
    //load problem data
    /*
    c_int n = 3;
    c_int m = 1;
    c_int P_nnz = 6;
    c_float P_x[6] = {1.0, 1.0, 2.0, 1.0, 1.0, 3.0};
    c_int P_i[6] = {0, 0, 1, 0, 1, 2};
    c_int P_p[4] = {0, 1, 3, 6};
    c_float q[3] = {0.0, 0.0, 0.0};
    c_int A_nnz = 3;
    c_float A_x[3] = {1.0, 1.0, 1.0};
    c_int A_i[3] = {0, 0, 0};
    c_int A_p[4] = {0, 1, 2, 3};
    c_float l[1] = {2.0};
    c_float u[1] = {2.0};
    */  
    // Load problem data
    
    c_float P_x[3] = {4.0, 1.0, 2.0, };
    c_int P_nnz = 3;
    c_int P_i[3] = {0, 0, 1, };
    c_int P_p[3] = {0, 1, 3, };
    c_float q[2] = {1.0, 1.0, };
    c_float A_x[4] = {1.0, 1.0, 1.0, 1.0, };
    c_int A_nnz = 4;
    c_int A_i[4] = {0, 1, 0, 2, };
    c_int A_p[3] = {0, 2, 4, };
    c_float l[3] = {1.0, 0.0, 0.0, };
    c_float u[3] = {1.0, 0.7, 0.7, };
    c_int n = 2;
    c_int m = 3;
    
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    if (data) {
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz, P_x, P_i, P_p);
        data->q = q;
        data->A = csc_matrix(data->m, data->n, A_nnz, A_x, A_i, A_p);
        data->l = l;
        data->u = u;
    }

    // Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
        settings->alpha = 1.0; // Change alpha parameter
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);
    c_float x[2] = {-1.2, 0.5};
    //osqp_warm_start_x(work, x);

    // Solve Problem
    osqp_solve(work);
    for(int i=0; i<n; i++){
        std::cout<<work->solution->x[i]<<" ";
    }

    // Cleanup
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);    
    std::cout<<"successfully run osqpTest"<<std::endl;
}