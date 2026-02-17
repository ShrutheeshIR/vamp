import numpy as np


def spherize_cuboid(cuboid):
    print(cuboid)
    min_diam = np.min(cuboid[3:6]) * 2.0

    # now actually fill these spheres
    spheres = []
    for i in range(int(cuboid[3] * 2 / min_diam)):
        for j in range(int(cuboid[4] * 2 / min_diam)):
            for k in range(int(cuboid[5] * 2 / min_diam)):
                spheres.append([
                        cuboid[0] - cuboid[3] + i * min_diam + min_diam/2,
                        cuboid[1] - cuboid[4] + j * min_diam + min_diam/2,
                        cuboid[2] - cuboid[5] + k * min_diam + min_diam/2,
                        min_diam/2
                    ]
                )

    return np.array(spheres)


if __name__ == "__main__":
    print(spherize_cuboid(np.array([0.0, 0.0, 0.0, 0.35, 0.15, 0.15])))
