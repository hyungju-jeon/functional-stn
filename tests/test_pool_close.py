import logging, multiprocessing, sys, time, signal


def pool_camera(camera_name: int):
    print(f'Running camera {camera_name}')
    while True:
        time.sleep(1)
        print(f'Camera {camera_name} still running')




if __name__ == "__main__":
    param_set = [(i,) for i in range(4)]
    d = [[]]*4

    for i in [0,1,2,3]:
        d[i] = multiprocessing.Process(target=pool_camera, args=param_set[i])
        d[i].daemon = True
        d[i].start()

    input("Press any key to terminate...")
    for i in [0, 1, 2, 3]:
        d[i].terminate()
