import pymcprotocol
import threading
import time
import socket
from typing import Optional, List, Dict, Any
from concurrent.futures import ThreadPoolExecutor, TimeoutError, Future
import logging

class PLCManager:
    """
    Singleton class để quản lý kết nối PLC với timeout handling
    """
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(PLCManager, cls).__new__(cls)
                    cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return
            
        self.plc = None
        self.is_connected = False
        self.connection_lock = threading.RLock()  # Sử dụng RLock thay vì Lock
        self.last_error = None
        self._initialized = True
        
        self.plc_ip = "192.168.5.10"
        self.plc_port = 5030
        self.plc_type = "iQ-R"
        
        self.connection_timeout = 5.0  
        self.operation_timeout = 3.0   
        self.socket_timeout = 2.0      
        
        self.executor = ThreadPoolExecutor(max_workers=5, thread_name_prefix="PLCManager")
        
        self.executor.submit(self._safe_connect)
    
    def _safe_connect(self) -> bool:
        """
        Kết nối an toàn với timeout
        """
        try:
            return self.connect()
        except Exception as e:
            logging.error(f"Safe connect failed: {e}")
            return False
    
    def connect(self) -> bool:
        """
        Kết nối đến PLC với timeout
        """
        try:
            # Sử dụng timeout cho lock
            if not self.connection_lock.acquire(timeout=self.connection_timeout):
                print("[PLC Manager] Connection lock timeout")
                return False
            
            try:
                if self.is_connected and self.plc:
                    return True
                
                self.plc = pymcprotocol.Type3E(plctype=self.plc_type)
                self.plc.setaccessopt(commtype="ascii")
                
                if hasattr(self.plc, 'socket') and self.plc.socket:
                    self.plc.socket.settimeout(self.socket_timeout)
                
                future = self.executor.submit(self._connect_internal)
                try:
                    result = future.result(timeout=self.connection_timeout)
                    if result:
                        self.is_connected = True
                        self.last_error = None
                        print(f"[PLC Manager] Connected to PLC at {self.plc_ip}:{self.plc_port}")
                        return True
                    else:
                        return False
                        
                except TimeoutError:
                    print(f"[PLC Manager] Connection timeout after {self.connection_timeout}s")
                    future.cancel()
                    self.last_error = f"Connection timeout after {self.connection_timeout}s"
                    return False
                    
            finally:
                self.connection_lock.release()
                
        except Exception as e:
            self.last_error = str(e)
            self.is_connected = False
            print(f"[PLC Manager] Connection failed: {e}")
            return False
    
    def _connect_internal(self) -> bool:
        """
        Thực hiện kết nối internal
        """
        try:
            self.plc.connect(self.plc_ip, self.plc_port)
            
            if hasattr(self.plc, 'socket') and self.plc.socket:
                self.plc.socket.settimeout(self.socket_timeout)
            
            return True
            
        except Exception as e:
            print(f"[PLC Manager] Internal connect error: {e}")
            return False
    
    def disconnect(self):
        """
        Ngắt kết nối PLC với timeout
        """
        try:
            if self.connection_lock.acquire(timeout=2.0):
                try:
                    if self.plc and self.is_connected:
                        try:
                            self.plc.close()
                            print("[PLC Manager] Disconnected from PLC")
                        except:
                            pass  # Ignore errors during disconnect
                finally:
                    self.is_connected = False
                    self.plc = None
                    self.connection_lock.release()
        except Exception as e:
            print(f"[PLC Manager] Error during disconnect: {e}")
            self.is_connected = False
            self.plc = None
    
    def reconnect(self) -> bool:
        """
        Kết nối lại PLC với timeout
        """
        print("[PLC Manager] Attempting to reconnect...")
        self.disconnect()
        time.sleep(0.5)  
        return self.connect()
    
    def _execute_with_timeout(self, func, *args, **kwargs):
        """
        Thực hiện function với timeout
        """
        future = self.executor.submit(func, *args, **kwargs)
        try:
            return future.result(timeout=self.operation_timeout)
        except TimeoutError:
            print(f"[PLC Manager] Operation timeout after {self.operation_timeout}s")
            future.cancel()
            self.is_connected = False  
            return None
    
    def read_random(self, word_devices: List[str] = None, dword_devices: List[str] = None) -> Optional[tuple]:
        """
        Đọc dữ liệu từ PLC với timeout và auto-reconnect
        """
        if not self.is_connected:
            if not self.reconnect():
                return None
        
        try:
            if not self.connection_lock.acquire(timeout=1.0):
                print("[PLC Manager] Read lock timeout")
                return None
            
            try:
                if word_devices is None:
                    word_devices = []
                if dword_devices is None:
                    dword_devices = []
                
                result = self._execute_with_timeout(
                    self._read_internal, 
                    word_devices, 
                    dword_devices
                )
                
                if result is not None:
                    return result
                    
            finally:
                self.connection_lock.release()
                
        except Exception as e:
            print(f"[PLC Manager] Read error: {e}")
            self.is_connected = False
        
        # Thử reconnect một lần nếu failed
        if self.reconnect():
            try:
                if self.connection_lock.acquire(timeout=1.0):
                    try:
                        result = self._execute_with_timeout(
                            self._read_internal, 
                            word_devices, 
                            dword_devices
                        )
                        return result
                    finally:
                        self.connection_lock.release()
            except Exception as e2:
                print(f"[PLC Manager] Read error after reconnect: {e2}")
        
        return None
    
    def _read_internal(self, word_devices: List[str], dword_devices: List[str]):
        """
        Internal read function
        """
        if not self.plc:
            raise Exception("PLC not connected")
            
        return self.plc.randomread(
            word_devices=word_devices, 
            dword_devices=dword_devices
        )
    
    def write_random(self, word_devices: List[str] = None, word_values: List[int] = None,
                    dword_devices: List[str] = None, dword_values: List[int] = None) -> bool:
        """
        Ghi dữ liệu vào PLC với timeout và auto-reconnect
        """
        if not self.is_connected:
            if not self.reconnect():
                return False
        
        try:
            if not self.connection_lock.acquire(timeout=1.0):
                print("[PLC Manager] Write lock timeout")
                return False
            
            try:
                if word_devices is None:
                    word_devices = []
                if word_values is None:
                    word_values = []
                if dword_devices is None:
                    dword_devices = []
                if dword_values is None:
                    dword_values = []
                
                # Thực hiện write với timeout
                result = self._execute_with_timeout(
                    self._write_internal,
                    word_devices, word_values,
                    dword_devices, dword_values
                )
                
                return result is not None
                
            finally:
                self.connection_lock.release()
                
        except Exception as e:
            print(f"[PLC Manager] Write error: {e}")
            self.is_connected = False
        
        if self.reconnect():
            try:
                if self.connection_lock.acquire(timeout=1.0):
                    try:
                        result = self._execute_with_timeout(
                            self._write_internal,
                            word_devices, word_values,
                            dword_devices, dword_values
                        )
                        return result is not None
                    finally:
                        self.connection_lock.release()
            except Exception as e2:
                print(f"[PLC Manager] Write error after reconnect: {e2}")
        
        return False
    
    def _write_internal(self, word_devices: List[str], word_values: List[int],
                       dword_devices: List[str], dword_values: List[int]):
        """
        Internal write function
        """
        if not self.plc:
            raise Exception("PLC not connected")
            
        self.plc.randomwrite(
            word_devices=word_devices,
            word_values=word_values,
            dword_devices=dword_devices, 
            dword_values=dword_values
        )
        return True
    
    def read_device_block(self, device_name: List[str], size: int) -> Optional[List[int]]:
        """
        Đọc block dữ liệu từ device với timeout
        """
        if not self.is_connected:
            if not self.reconnect():
                return None
        
        try:
            if not self.connection_lock.acquire(timeout=1.0):
                print("[PLC Manager] Block read lock timeout")
                return None
            
            try:
                result = self._execute_with_timeout(
                    self._read_block_internal,
                    device_name, size
                )
                return result
            finally:
                self.connection_lock.release()
                
        except Exception as e:
            print(f"[PLC Manager] Block read error: {e}")
            self.is_connected = False
            return None
    
    def _read_block_internal(self, device_name: List[str], size: int):
        """
        Internal block read function
        """
        if not self.plc:
            raise Exception("PLC not connected")
            
        return self.plc.batchread_wordunits(
            headdevice=device_name, 
            readsize=size
        )
    
    def write_device_block(self, device_name: List[str], values: List[int]) -> bool:
        """
        Ghi block dữ liệu vào device với timeout
        """
        if not self.is_connected:
            if not self.reconnect():
                return False
        
        try:
            if not self.connection_lock.acquire(timeout=1.0):
                print("[PLC Manager] Block write lock timeout")
                return False
            
            try:
                result = self._execute_with_timeout(
                    self._write_block_internal,
                    device_name, values
                )
                return result is not None
            finally:
                self.connection_lock.release()
                
        except Exception as e:
            print(f"[PLC Manager] Block write error: {e}")
            self.is_connected = False
            return False
    
    def _write_block_internal(self, device_name: List[str], values: List[int]):
        """
        Internal block write function
        """
        if not self.plc:
            raise Exception("PLC not connected")
            
        self.plc.randomwrite_bitunits(
            bit_devices=device_name, 
            values=values
        )
        return True
    
    def get_status(self) -> Dict[str, Any]:
        """
        Lấy trạng thái kết nối PLC
        """
        return {
            "connected": self.is_connected,
            "plc_ip": self.plc_ip,
            "plc_port": self.plc_port,
            "last_error": self.last_error,
            "connection_timeout": self.connection_timeout,
            "operation_timeout": self.operation_timeout
        }
    
    def set_timeouts(self, connection_timeout: float = None, operation_timeout: float = None, socket_timeout: float = None):
        """
        Cấu hình timeout
        """
        if connection_timeout is not None:
            self.connection_timeout = connection_timeout
        if operation_timeout is not None:
            self.operation_timeout = operation_timeout
        if socket_timeout is not None:
            self.socket_timeout = socket_timeout
    
    def __del__(self):
        """
        Cleanup khi object bị destroy
        """
        try:
            self.disconnect()
            if hasattr(self, 'executor'):
                self.executor.shutdown(wait=False)
        except:
            pass

plc_manager = PLCManager()

def get_plc_manager() -> PLCManager:
    """
    Helper function để lấy PLC Manager instance
    """
    return plc_manager