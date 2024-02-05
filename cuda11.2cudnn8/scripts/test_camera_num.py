import cv2

def main():

    # for i in range(100):
    #     # 開啟 webcam
    #     cap = cv2.VideoCapture(i)

    #     # 檢查 webcam 是否成功打開
    #     try:
    #         if not cap.isOpened():
    #             print("Error: Could not open webcam.")
    #             continue
    #         else:
    #             print("Success: Open webcam", i)
    #             break
    #     except Exception as e:
    #         print(e)

    # 開啟 webcam
    cap = cv2.VideoCapture(4)

    # 檢查 webcam 是否成功打開
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while True:
        # 讀取一幀影像
        ret, frame = cap.read()

        # 檢查影像是否成功讀取
        if not ret:
            print("Error: Could not read frame.")
            break

        # 顯示即時影像
        cv2.imshow('Webcam', frame)

        # 檢查按鍵 'q' 是否被按下，若是則跳出迴圈
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 釋放 webcam 資源及關閉視窗
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
