import multiprocessing
import threading
from ctypes import  c_double,c_float



class DoubleBuffer():
  def __init__(self,init_val,t=c_float,name=""):
    manager = multiprocessing.Manager()
    self.buffer_a = multiprocessing.Array(t,init_val)
    self.buffer_b = multiprocessing.Array(t,init_val)
    self.buffer_switch = multiprocessing.Value('b',True)
    self.lock = threading.Lock()
    self.read_count=0
    self.write_count=0
    self.name=name


  def put(self,value):
    # self.write_count+=1
    # if self.write_count %100 == 0:
    #   print(self.name," write count: ",self.write_count)
    if self.buffer_switch.value:
      self.buffer_a[:] = value
    else:
      self.buffer_b[:]=value
    with self.lock:
      self.buffer_switch.value = not self.buffer_switch.value


  def get(self):
    # self.read_count+=1
    # if self.read_count %100==0:
    #   print(self.name, " read count: ",self.read_count)
    with self.lock:
      if self.buffer_switch.value:
        return tuple(self.buffer_b)
      else:
        return tuple(self.buffer_a)
