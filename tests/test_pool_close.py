import logging, multiprocessing, sys, time, signal


def pool_camera(camera_name: int, n):
    camera = test(camera_name)
    print(f'current state is {n.value}')
    print(f'Running camera {camera_name}')
    camera.camera_run()


class test:
    def __init__(self, cam_name: int):
        self.name = cam_name
        self.run = True
        signal.signal(signal.SIGINT, self.close_pipeline)
        signal.signal(signal.SIGTERM, self.close_pipeline)

    def close_pipeline(self, *args):
        print('Closing well!')
        self.run = False

    def camera_run(self):
        while self.run:
            time.sleep(1)
            # print(f'Camera {self.name} still running')



if __name__ == "__main__":
    param_set = [0, 1, 2, 3]
    num = multiprocessing.Value('d', 0.0)
    d = [[]] * 4

    for i in [0, 1, 2, 3]:
        d[i] = multiprocessing.Process(target=pool_camera, args=(param_set[i], num))
        d[i].daemon = True
        d[i].start()

    input("Press any key to terminate...")
    num.value = 2

    input("Press any key to terminate...")
    for i in [0, 1, 2, 3]:
        d[i].terminate()