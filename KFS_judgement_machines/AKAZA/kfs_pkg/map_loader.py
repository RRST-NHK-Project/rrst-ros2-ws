import cv2
import os

class KFSMapLoader:
    def __init__(self, akaze_obj):
        self.akaze = akaze_obj
        self.map_list = []

    def load_all_pages(self, directory_path):
        """
        ディレクトリ内の p1.png ~ p22.png を全て読み込み、特徴量をリスト化する
        """
        self.map_list = []
        # PDFのページ数（1〜22）に合わせてループ
        for i in range(1, 23):
            file_name = f"p{i}.png"
            path = os.path.join(directory_path, file_name)
            
            img = cv2.imread(path, 0) # グレースケールで読み込み
            if img is None:
                print(f"[Skip] {file_name} が見つかりません。")
                continue
                
            # 各ページの特徴量を計算
            kp, des = self.akaze.detectAndCompute(img, None)
            
            # ページ情報として辞書形式で格納
            self.map_list.append({
                "page_id": i,
                "img": img,
                "kp": kp,
                "des": des,
                "h": img.shape[0],
                "w": img.shape[1]
            })
            
        print(f"--- Map Load Complete: {len(self.map_list)} pages registered ---")
        return self.map_list