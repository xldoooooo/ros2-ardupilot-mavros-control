import cv2

cuda_stream = cv2.cuda_Stream()

cap = cv2.VideoCapture("/home/zzx/vid.mp4")

if not cap.isOpened():
    print("cam can not open")
    exit()

while True:
    ret, frame = cap.read()

    if ret:
        gpu_frame = cv2.cuda_GpuMat()
        gpu_frame.upload(frame,cuda_stream)

        gpu_gray = cv2.cuda.cvtColor(gpu_frame,cv2.COLOR_BGR2GRAY, stream = cuda_stream)

        output_frame = gpu_gray.download(cuda_stream)

        cv2.imshow('Camera',frame)

        cv2.imshow('GPU Camera',output_frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    else:
        print("fail")
        break

cap.release()
cv2.destroyAllWindows()