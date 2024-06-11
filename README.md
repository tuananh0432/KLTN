#rainfall_pH_IoT
Chức năng chính:
1.	Đo pH: 
	Đọc giá trị analog từ cảm biến pH
	Chuyển đổi giá trị analog thành pH
	Lưu trữ giá trị pH vào mảng để tính trung bình
2.	Đo lượng mưa: 
	Đếm xung từ cảm biến mưa
	Chuyển đổi số xung thành lượng mưa
3.	Gửi dữ liệu lên ThingSpeak: 
	Kết nối WiFi
	Gửi giá trị pH và lượng mưa lên kênh ThingSpeak
4.	Cảnh báo qua SMS: 
	Nếu lượng mưa vượt quá ngưỡng hoặc pH quá thấp/cao, gửi SMS cảnh báo đến số điện thoại được cài đặt.
	Nếu đã gửi cảnh báo, lấy vị trí GPS và gửi kèm theo trong tin nhắn SMS.
5.	GPS: 
	Bật/tắt GPS khi cần thiết để tiết kiệm năng lượng.
	Lấy vị trí GPS khi cần gửi cảnh báo.
