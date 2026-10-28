#include <epicsUnitTest.h>
#include <testMain.h>

#include "trajectory.hpp"

#include <iostream>

template <typename Container>
void print_vector(const Container& vec) {
    std::cout << "[";
    for (size_t i = 0; i < vec.size(); i++) {
        if (i == vec.size()-1) {
            std::cout << vec[i] << "]\n";
        } else {
            std::cout << vec[i] << " ";
        }
    }
}


MAIN(testCollisions) {
    testPlan(0);

    std::vector<double> sparse_traj = {
        0.0000,-1.5700,-1.5700,-1.4000,-1.5700,1.0100,-0.0200,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
        1.0100,-1.2500,-1.2500,-1.6500,-1.3000,1.2000,0.1500,0.3564,0.0594,-0.2228,0.2574,-0.0545,0.1832,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
        2.0200,-0.8500,-1.4500,-1.8500,-1.0500,0.9000,0.3500,0.3465,0.0495,0.0495,-0.0248,-0.0495,-0.0248,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,
        3.0300,-0.5500,-1.1500,-1.5500,-1.3500,1.1000,0.1000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000
    };

    auto [traj, msg] = build_quintic_trajectory(sparse_traj);
    testOk1(traj.has_value());
    for (const auto& p : traj->points) {
        printf("Time = %lf\n", p.time);
        print_vector(p.position);
        print_vector(p.velocity);
        print_vector(p.acceleration);
        printf("\n");
    }

    {
        std::vector<int> vec = {1, 2, 3, 4, 5, 6};
        auto unflattened_vec = unflatten(vec, 3);
        testOk1(unflattened_vec.has_value());
        testOk1(unflattened_vec->size() == 2);
        testOk1(unflattened_vec.value()[0].size() == 3);
    }

    {
        std::vector<std::vector<int>> vec = {
            {1,2,3},
            {4,5,6}
        };
        std::vector<int> unflattened_vec = flatten(vec);
        testOk1(unflattened_vec.size() == 6);
        testOk1(unflattened_vec.front() == 1);
        testOk1(unflattened_vec.back() == 6);
    }

    return testDone();
}
