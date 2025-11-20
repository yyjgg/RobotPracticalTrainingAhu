"""
网格状迷宫地图生成器
生成类似参考图的矩形障碍物迷宫地图
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import ListedColormap
import random

class GridMapGenerator:
    def __init__(self, width=20, height=15, cell_size=50):
        """
        初始化网格地图生成器
        
        Args:
            width (int): 地图宽度（单元格数量）
            height (int): 地图高度（单元格数量）
            cell_size (int): 每个单元格的像素大小
        """
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.grid = None
        self.start_pos = None
        self.end_pos = None
        self.path = []
        
        # 定义颜色
        self.colors = {
            'background': '#f0f8ff',      # 浅蓝色背景
            'obstacle': '#000000',        # 黑色障碍物
            'start': '#ff6b6b',           # 红色起点
            'end': '#4ecdc4',             # 青色终点
            'path': '#d9480f',            # 橙色路径
            'path_border': '#9c27b0'      # 紫色路径边框
        }
    
    def generate_map(self, obstacle_density=0.25, min_obstacle_size=2, max_obstacle_size=5):
        """
        生成网格地图
        
        Args:
            obstacle_density (float): 障碍物密度（0-1之间）
            min_obstacle_size (int): 最小障碍物大小
            max_obstacle_size (int): 最大障碍物大小
        """
        # 创建空网格
        self.grid = np.zeros((self.height, self.width), dtype=int)
        
        # 生成障碍物
        obstacles_placed = 0
        total_cells = self.width * self.height
        target_obstacles = int(total_cells * obstacle_density)
        
        while obstacles_placed < target_obstacles:
            # 随机选择障碍物大小
            size_w = random.randint(min_obstacle_size, max_obstacle_size)
            size_h = random.randint(min_obstacle_size, max_obstacle_size)
            
            # 随机选择位置
            x = random.randint(0, self.width - size_w)
            y = random.randint(0, self.height - size_h)
            
            # 检查是否与已有障碍物重叠或太靠近边界
            if self._is_valid_obstacle_position(x, y, size_w, size_h):
                # 放置障碍物
                self.grid[y:y+size_h, x:x+size_w] = 1
                obstacles_placed += size_w * size_h
        
        # 设置起点和终点
        self._set_start_and_end_points()
        
        return self.grid
    
    def _is_valid_obstacle_position(self, x, y, w, h):
        """检查障碍物位置是否有效"""
        # 检查是否超出边界
        if x < 0 or y < 0 or x + w > self.width or y + h > self.height:
            return False
        
        # 检查是否与已有障碍物重叠
        if np.any(self.grid[y:y+h, x:x+w] == 1):
            return False
        
        # 检查是否太靠近起点/终点区域
        if (x < 3 and y < 3) or (x + w > self.width - 3 and y + h > self.height - 3):
            return False
        
        return True
    
    def _set_start_and_end_points(self):
        """设置起点和终点"""
        # 起点设置在左上角附近
        self.start_pos = (0, 0)
        self.grid[self.start_pos[1]][self.start_pos[0]] = 2
        
        # 终点设置在右下角附近
        self.end_pos = (self.width - 1, self.height - 1)
        self.grid[self.end_pos[1]][self.end_pos[0]] = 3
    
    def find_path(self):
        """使用简化的A*算法寻找路径"""
        if self.grid is None:
            raise ValueError("地图尚未生成")
        
        # 简单实现，实际应该使用更复杂的寻路算法
        # 这里我们生成一个示例路径
        self.path = self._generate_sample_path()
        return self.path
    
    def _generate_sample_path(self):
        """生成示例路径"""
        path = []
        
        # 从起点开始
        x, y = self.start_pos
        
        # 简单的之字形路径
        while x < self.width - 1 or y < self.height - 1:
            path.append((x, y))
            
            # 尝试向右移动
            if x < self.width - 1 and self.grid[y][x + 1] != 1:
                x += 1
            # 尝试向下移动
            elif y < self.height - 1 and self.grid[y + 1][x] != 1:
                y += 1
            # 尝试向左移动（回溯）
            elif x > 0 and self.grid[y][x - 1] != 1:
                x -= 1
            # 尝试向上移动（回溯）
            elif y > 0 and self.grid[y - 1][x] != 1:
                y -= 1
            else:
                break
        
        # 添加终点
        path.append(self.end_pos)
        return path
    
    def plot_map(self, show_path=True, save_path=None):
        """
        绘制地图
        
        Args:
            show_path (bool): 是否显示路径
            save_path (str): 保存路径，如果为None则不保存
        """
        if self.grid is None:
            raise ValueError("地图尚未生成")
        
        # 创建图形
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # 设置图形大小
        fig.set_size_inches(self.width * 0.8, self.height * 0.8)
        
        # 绘制背景
        ax.set_facecolor(self.colors['background'])
        
        # 绘制网格线
        for i in range(self.width + 1):
            ax.axvline(x=i, color='gray', linewidth=0.5, alpha=0.3)
        for i in range(self.height + 1):
            ax.axhline(y=i, color='gray', linewidth=0.5, alpha=0.3)
        
        # 绘制障碍物
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] == 1:  # 障碍物
                    rect = patches.Rectangle(
                        (x, y), 1, 1,
                        linewidth=1,
                        edgecolor='gray',
                        facecolor=self.colors['obstacle']
                    )
                    ax.add_patch(rect)
                elif self.grid[y][x] == 2:  # 起点
                    rect = patches.Rectangle(
                        (x, y), 1, 1,
                        linewidth=2,
                        edgecolor='darkred',
                        facecolor=self.colors['start']
                    )
                    ax.add_patch(rect)
                elif self.grid[y][x] == 3:  # 终点
                    rect = patches.Rectangle(
                        (x, y), 1, 1,
                        linewidth=2,
                        edgecolor='darkcyan',
                        facecolor=self.colors['end']
                    )
                    ax.add_patch(rect)
        
        # 绘制路径
        if show_path and self.path:
            # 绘制路径线
            path_x = [p[0] + 0.5 for p in self.path]
            path_y = [p[1] + 0.5 for p in self.path]
            
            # 绘制路径边框
            ax.plot(path_x, path_y, 
                   color=self.colors['path_border'], 
                   linewidth=4, 
                   alpha=0.8)
            
            # 绘制路径主线
            ax.plot(path_x, path_y, 
                   color=self.colors['path'], 
                   linewidth=2, 
                   alpha=1.0)
        
        # 设置坐标轴
        ax.set_xlim(0, self.width)
        ax.set_ylim(0, self.height)
        ax.set_aspect('equal')
        
        # 反转y轴，使(0,0)在左上角
        ax.invert_yaxis()
        
        # 隐藏坐标轴标签
        ax.set_xticks([])
        ax.set_yticks([])
        
        # 添加标题
        ax.set_title('网格迷宫地图', fontsize=16, fontweight='bold', pad=20)
        
        # 调整布局
        plt.tight_layout()
        
        # 保存图片
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"地图已保存到: {save_path}")
        
        # 显示图片
        plt.show()

def generate_single_map_example():
    """生成单个地图示例"""
    print("=== 网格迷宫地图生成器 ===")
    print("正在生成地图...")
    
    # 创建地图生成器
    map_gen = GridMapGenerator(width=25, height=20)
    
    # 生成地图
    map_gen.generate_map(
        obstacle_density=0.22,
        min_obstacle_size=2,
        max_obstacle_size=4
    )
    
    # 寻找路径
    map_gen.find_path()
    
    # 绘制地图
    map_gen.plot_map(
        show_path=True,
        save_path="grid_maze_map.png"
    )
    
    print("地图生成完成！")
    print("文件已保存为: grid_maze_map.png")

if __name__ == "__main__":
    generate_single_map_example()
