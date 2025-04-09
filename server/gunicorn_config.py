bind = "0.0.0.0:8000"      
workers = 4               # Số worker = (2 * CPU cores) + 1
threads = 2                 
worker_class = "gthread"    
timeout = 120               
accesslog = "-"
errorlog = "-"              

import logging

loglevel = "error"  # Chỉ log lỗi
errorlog = "gunicorn-error.log"  # Ghi log lỗi vào file
accesslog = None  # Tắt access log

# # Tạo bộ lọc log tùy chỉnh
# class FilterOutInfo(logging.Filter):
#     def filter(self, record):
#         return record.levelno > logging.INFO  # Chỉ log từ WARNING trở lên

# # Thêm bộ lọc vào Gunicorn
# def on_starting(server):
#     logger = logging.getLogger('gunicorn.error')
#     logger.addFilter(FilterOutInfo())