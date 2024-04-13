from control import EETaskHandler
from perception import process

def main():
    try:
        handler = EETaskHandler()

        # Get positions of blocks and pieces
        poses, centroids, blocks = process()

        finished_building = False

        # 1. Clear space around starting frame for construction
        
        # 2. Begin loop
        while not finished_building:
            pass
    
    finally:
        handler.cleanup()


if __name__ == '__main__':
    main()