import pygame
import time

def main():
    # 初始化Pygame
    pygame.init()
    screen = pygame.display.set_mode((500, 350))
    pygame.display.set_caption("XYZ值控制")
    font = pygame.font.SysFont(None, 36)
    small_font = pygame.font.SysFont(None, 28)
    
    # 初始化XYZ值为0
    x = 0.0
    y = 0.0
    z = 0.0
    
    # 步长
    step_size = 0.01
    
    print("控制说明:")
    print("W/S: 增加/减少 X值")
    print("A/D: 增加/减少 Y值")
    print("↑/↓: 增加/减少 Z值")
    print("R: 重置所有值为0")
    print("Q或ESC: 退出")
    
    running = True
    clock = pygame.time.Clock()
    
    while running:
        # 处理事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE or event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_r:
                    x = 0.0
                    y = 0.0
                    z = 0.0
                    print("已重置所有值为0")
        
        # 获取按键状态
        keys = pygame.key.get_pressed()
        
        # 控制X值
        if keys[pygame.K_w]:
            x += step_size
        if keys[pygame.K_s]:
            x -= step_size
        
        # 控制Y值
        if keys[pygame.K_d]:
            y += step_size
        if keys[pygame.K_a]:
            y -= step_size
        
        # 控制Z值
        if keys[pygame.K_UP]:
            z += step_size
        if keys[pygame.K_DOWN]:
            z -= step_size
        
        # 绘制界面
        screen.fill((30, 30, 40))
        font_path = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
        font = pygame.font.Font(font_path, 16)
        small_font = pygame.font.Font(font_path, 12)
        
        # 标题
        title = font.render("XYZ值控制", True, (255, 255, 255))
        screen.blit(title, (150, 30))
        
        # 显示X值
        x_text = font.render(f"X: {x:.3f}", True, (255, 100, 100))
        screen.blit(x_text, (50, 100))
        x_control = small_font.render("(W/S)", True, (200, 200, 200))
        screen.blit(x_control, (300, 105))
        
        # 显示Y值
        y_text = font.render(f"Y: {y:.3f}", True, (100, 255, 100))
        screen.blit(y_text, (50, 160))
        y_control = small_font.render("(A/D)", True, (200, 200, 200))
        screen.blit(y_control, (300, 165))
        
        # 显示Z值
        z_text = font.render(f"Z: {z:.3f}", True, (100, 150, 255))
        screen.blit(z_text, (50, 220))
        z_control = small_font.render("(↑/↓)", True, (200, 200, 200))
        screen.blit(z_control, (300, 225))
        
        # 底部提示
        hint = small_font.render("R:重置  Q/ESC:退出", True, (150, 150, 150))
        screen.blit(hint, (120, 290))
        
        pygame.display.flip()
        clock.tick(60)  # 60 FPS
        
        # 输出到控制台（可选）
        # print(f"X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")
    
    # 清理
    pygame.quit()
    print(f"\n最终值: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

if __name__ == "__main__":
    main()