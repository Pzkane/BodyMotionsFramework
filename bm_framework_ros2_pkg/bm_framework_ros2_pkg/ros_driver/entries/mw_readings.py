import subprocess
from time import sleep

def main():
    process_run_successful = False
    while not process_run_successful:
        try:
            subprocess.run(["ros2", "run", "bm_framework_ros2_pkg", "bm_metawear_slave"],
                           check=True)
        except subprocess.CalledProcessError:
            print("probably segfaulted")
            sleep(1)
        else:
            print("ok")
            process_run_successful = True

if __name__ == "__main__":
    main()
