import cv2
import time
import os

this_file = os.getcwd()
IMG_DIR = this_file + '/img/'

def main():
    TIMER = int(5)

    # Open the camera
    cap = cv2.VideoCapture(0)

    print("Press y to take image.")
    not_end = False
    while not not_end:
        
        # Read and display each frame
        ret, img = cap.read()
        cv2.imshow('a', img)

        # check for the key pressed
        k = cv2.waitKey(125)

        # set the key for the countdown
        if k == ord('y'):
            prev = time.time()

            while TIMER >= 0:
                ret, img = cap.read()

                # Display countdown on each frame
                # specify the font and draw the
                # countdown using puttext
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(img, str(TIMER),
                            (200, 250), font,
                            7, (0, 0, 255),
                            4, cv2.LINE_AA)
                cv2.imshow('a', img)
                cv2.waitKey(125)

                # current time
                cur = time.time()

                # Update and keep track of Countdown
                # if time elapsed is one second
                # then decrease the counter
                if cur-prev >= 1:
                    prev = cur
                    TIMER = TIMER-1

            else:
                ret, img = cap.read()

                # Display the clicked frame for 2
                # sec.You can increase time in
                # waitKey also
                cv2.imshow('a', img)

                # time for which image displayed
                cv2.waitKey(2000)

                # Save the frame
                save_name = input("Enter the name of the file:")
                filename = IMG_DIR + save_name + '.jpg'
                cv2.imwrite(filename, img)

                # HERE we can reset the Countdown timer
                # if we want more Capture without closing
                # the camera
                cap.release()
                cv2.destroyAllWindows()
                return img, save_name

        # # Press Esc to exit
        # elif k == ord('q'):
        #     not_end = True
        #     break

    # close the camera
    cap.release()

    # close all the opened windows
    cv2.destroyAllWindows()
    return img, save_name
