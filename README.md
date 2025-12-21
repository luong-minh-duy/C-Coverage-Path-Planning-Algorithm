C* Coverage Path Planning – Experimental Implementation

Dự án này hiện thực và thử nghiệm giải thuật C*
(Coverage Path Planning for Unknown Environments using Rapidly Covering Graphs),
dựa trên bài báo của Z. Shen, J. P. Wilson, S. Gupta.

Mục tiêu của project:

Hiện thực giải thuật C* trong môi trường 2D

Thử nghiệm trên nhiều testcase khác nhau

Đánh giá khả năng bao phủ và các giới hạn của giải thuật

Phục vụ cho báo cáo học phần / nghiên cứu

1. Yêu cầu môi trường

Python >= 3.8

Các thư viện Python cần thiết:

pip install numpy shapely networkx matplotlib

2. Cấu trúc thư mục (tham khảo)
.
├── ....
├── main.py
├── testcase_1.json
├── testcase_2.json
├── testcase_3.json
├── testcase_4.json
├── testcase_5.json
├── README.md

3. Cách chạy chương trình

Chạy chương trình bằng lệnh:

python main.py


Chương trình sẽ:

Load một testcase từ file JSON

Chạy giải thuật C*

In ra thông tin thực nghiệm (iteration, path length, coverage ratio)

Vẽ quỹ đạo coverage và Rapidly Covering Graph (RCG)

4. Thay đổi testcase đầu vào

Trong file main.py, tìm đoạn code:

env, obstacles, start = load_case_from_json("testcase_5.json")


Thay "testcase_5.json" bằng testcase khác, ví dụ:

load_case_from_json("testcase_1.json")
load_case_from_json("testcase_2.json")
load_case_from_json("testcase_3.json")
load_case_from_json("testcase_4.json")


Mỗi file testcase mô tả:

Hình dạng môi trường

Danh sách chướng ngại vật

Vị trí xuất phát của robot

5. Thay đổi tham số của giải thuật

Giải thuật C* được gọi trong main.py như sau:

path, G, env_free, covered = C_star_coverage(
    env,
    obstacles,
    start_pos=start,
    sensor_range=6.0,
    cover_radius=5.5,
    d_s=5.0,
    verbose=True
)

Ý nghĩa các tham số

sensor_range
Phạm vi cảm biến của robot, ảnh hưởng đến quá trình khám phá môi trường.

cover_radius
Bán kính bao phủ hình học (coverage footprint), ảnh hưởng trực tiếp đến coverage ratio.

d_s
Khoảng cách giữa các lap (lap spacing).
Theo bài báo C*, nên thỏa điều kiện:

d_s ≤ 2 * cover_radius


để đảm bảo vùng giữa các lap được bao phủ.

verbose

True: in chi tiết từng iteration

False: chỉ in kết quả cuối

Bộ tham số khuyến nghị
sensor_range = 6.0
cover_radius = 5.5
d_s = 5.0

6. Kết quả đầu ra

Sau khi chạy, chương trình sẽ in ra:

Số vòng lặp thực thi

Số node và edge của Rapidly Covering Graph (RCG)

Độ dài quỹ đạo robot

Coverage ratio (%)

Đồng thời hiển thị hình ảnh gồm:

Quỹ đạo coverage (màu xanh)

Rapidly Covering Graph (màu đỏ)

Vùng đã bao phủ (màu cyan)

Chướng ngại vật (màu xám)

7. Lưu ý

Đây là mô phỏng của giải thuật C*, không phải hiện thực đầy đủ như trong bài báo gốc.

Coverage ratio được đo theo diện tích hình học, trong khi C* đảm bảo coverage theo nghĩa topo.

Trong môi trường phức tạp, coverage ratio có thể thấp hơn; các giới hạn này đã được phân tích trong báo cáo.

8. Tài liệu tham khảo

Z. Shen, J. P. Wilson, S. Gupta
C*: A Coverage Path Planning Algorithm for Unknown Environments using Rapidly Covering Graphs