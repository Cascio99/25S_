import psutil
import time
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import re
import threading
from datetime import datetime

class FasterLIOMonitor:
    def __init__(self):
        # 모니터링할 프로세스 키워드들
        self.process_keywords = {
            'roslaunch': 'ROS Launch',
            'rosmaster': 'ROS Master', 
            'rosout': 'ROS Out',
            'faster_lio': 'FASTER-LIO Core',
            'run_mapping_online': 'Laser Mapping',
            'rviz': 'RViz Visualization',
            'rosbag': 'ROS Bag Player'
        }
        
        # 데이터 저장용 딕셔너리
        self.process_data = defaultdict(lambda: {'cpu': [], 'memory': [], 'timestamps': []})
        self.monitoring = False
        
    def find_processes(self):
        """관련 프로세스들을 찾습니다"""
        found_processes = {}
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline']) if proc.info['cmdline'] else ''
                
                for keyword, display_name in self.process_keywords.items():
                    if keyword in cmdline.lower() or keyword in proc.info['name'].lower():
                        if display_name not in found_processes:
                            found_processes[display_name] = []
                        found_processes[display_name].append(proc.info['pid'])
                        
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                continue
                
        return found_processes
    
    def get_process_stats(self, pid):
        """특정 프로세스의 CPU와 메모리 사용률을 가져옵니다"""
        try:
            proc = psutil.Process(pid)
            cpu_percent = proc.cpu_percent()
            memory_info = proc.memory_info()
            memory_mb = memory_info.rss / 1024 / 1024  # MB 단위
            return cpu_percent, memory_mb
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            return None, None
    
    def monitor_processes(self, duration=60, interval=1):
        """프로세스들을 모니터링합니다"""
        print(f"FASTER-LIO 프로세스 모니터링 시작 (지속시간: {duration}초, 간격: {interval}초)")
        
        self.monitoring = True
        start_time = time.time()
        
        while self.monitoring and (time.time() - start_time) < duration:
            current_processes = self.find_processes()
            timestamp = datetime.now()
            
            for process_name, pids in current_processes.items():
                total_cpu = 0
                total_memory = 0
                active_pids = 0
                
                for pid in pids:
                    cpu, memory = self.get_process_stats(pid)
                    if cpu is not None and memory is not None:
                        total_cpu += cpu
                        total_memory += memory
                        active_pids += 1
                
                if active_pids > 0:
                    avg_cpu = total_cpu / active_pids if active_pids > 1 else total_cpu
                    self.process_data[process_name]['cpu'].append(avg_cpu)
                    self.process_data[process_name]['memory'].append(total_memory)
                    self.process_data[process_name]['timestamps'].append(timestamp)
            
            time.sleep(interval)
        
        print("모니터링 완료!")
    
    def create_summary_table(self):
        """요약 테이블을 생성합니다"""
        summary_data = []
        
        for process_name, data in self.process_data.items():
            if data['cpu']:  # 데이터가 있는 경우만
                cpu_avg = np.mean(data['cpu'])
                cpu_max = np.max(data['cpu'])
                memory_avg = np.mean(data['memory'])
                memory_max = np.max(data['memory'])
                
                summary_data.append({
                    'Process': process_name,
                    'CPU 평균 (%)': round(cpu_avg, 2),
                    'CPU 최대 (%)': round(cpu_max, 2),
                    'RAM 평균 (MB)': round(memory_avg, 2),
                    'RAM 최대 (MB)': round(memory_max, 2),
                    '샘플 수': len(data['cpu'])
                })
        
        return pd.DataFrame(summary_data)
    
    def plot_usage_graphs(self, save_plots=True):
        """CPU와 RAM 사용률 그래프를 생성합니다"""
        if not self.process_data:
            print("표시할 데이터가 없습니다.")
            return
        
        # 한글 폰트 설정 (시스템에 따라 조정 필요)
        plt.rcParams['font.family'] = ['DejaVu Sans', 'Arial Unicode MS', 'Malgun Gothic']
        plt.rcParams['axes.unicode_minus'] = False
        
        # CPU 사용률 그래프
        plt.figure(figsize=(15, 10))
        
        plt.subplot(2, 1, 1)
        for process_name, data in self.process_data.items():
            if data['cpu']:
                plt.plot(data['timestamps'], data['cpu'], 
                        marker='o', markersize=3, label=process_name, linewidth=2)
        
        plt.title('CPU Usage', fontsize=14, fontweight='bold')
        plt.xlabel('Time')
        plt.ylabel('CPU Usage (%)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.xticks(rotation=45)
        
        # RAM 사용률 그래프
        plt.subplot(2, 1, 2)
        for process_name, data in self.process_data.items():
            if data['memory']:
                plt.plot(data['timestamps'], data['memory'], 
                        marker='s', markersize=3, label=process_name, linewidth=2)
        
        plt.title('RAM Usage', fontsize=14, fontweight='bold')
        plt.xlabel('Time')
        plt.ylabel('RAM Usage (MB)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.xticks(rotation=45)
        
        plt.tight_layout()
        
        if save_plots:
            plt.savefig('fasterlio_usage_plots.png', dpi=300, bbox_inches='tight')
            print("그래프가 'fasterlio_usage_plots.png'로 저장되었습니다.")
        
        plt.show()
    
    def stop_monitoring(self):
        """모니터링을 중단합니다"""
        self.monitoring = False
    
    def export_raw_data(self, filename='fasterlio_raw_data.csv'):
        """원시 데이터를 CSV로 내보냅니다"""
        all_data = []
        
        for process_name, data in self.process_data.items():
            for i in range(len(data['cpu'])):
                all_data.append({
                    'Process': process_name,
                    'Timestamp': data['timestamps'][i],
                    'CPU (%)': data['cpu'][i],
                    'RAM (MB)': data['memory'][i]
                })
        
        df = pd.DataFrame(all_data)
        df.to_csv(filename, index=False)
        print(f"원시 데이터가 '{filename}'로 저장되었습니다.")

def main():
    # 모니터 인스턴스 생성
    monitor = FasterLIOMonitor()
    
    # 현재 실행 중인 관련 프로세스 확인
    print("=== 현재 실행 중인 FASTER-LIO 관련 프로세스 ===")
    current_processes = monitor.find_processes()
    
    if not current_processes:
        print("FASTER-LIO 관련 프로세스를 찾을 수 없습니다.")
        print("다음 프로세스들을 찾고 있습니다:")
        for keyword, name in monitor.process_keywords.items():
            print(f"  - {name} ({keyword})")
        return
    
    for process_name, pids in current_processes.items():
        print(f"{process_name}: PIDs {pids}")
    
    print("\n=== 모니터링 시작 ===")
    
    # 모니터링 실행 (60초간, 1초 간격)
    # 원하는 시간으로 조정 가능
    try:
        monitor.monitor_processes(duration=420, interval=60)
        
        # 결과 테이블 생성 및 출력
        print("\n=== 프로세스 사용률 요약 ===")
        summary_df = monitor.create_summary_table()
        print(summary_df.to_string(index=False))
        
        # 테이블을 CSV로 저장
        summary_df.to_csv('fasterlio_summary.csv', index=False)
        print(f"\n요약 테이블이 'fasterlio_summary.csv'로 저장되었습니다.")
        
        # 그래프 생성
        monitor.plot_usage_graphs()
        
        # 원시 데이터 저장
        monitor.export_raw_data()
        
    except KeyboardInterrupt:
        print("\n모니터링이 사용자에 의해 중단되었습니다.")
        monitor.stop_monitoring()

if __name__ == "__main__":
    main()