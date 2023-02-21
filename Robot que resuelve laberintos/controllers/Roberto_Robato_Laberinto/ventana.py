# -*- coding: utf-8 -*-
"""
Created on Tue Dec 14 16:22:26 2021

@author: Private Richi
"""

import pygame
import pickle
import numpy as np
from queue import PriorityQueue

def run_a_star(start_j,start_i):
    print("Iniciando algoritmo A*")
    print("Valores utilizados: ")
    print("y: {:<3}   x: {:<3}".format(start_i, start_j))
    
    pickle_off = open('mapa.txt', "rb")
    mapa = pickle.load(pickle_off)
    
    for i in range(mapa.shape[0]-2):
        for j in range(mapa.shape[1]-2):
            if mapa[i,j] == 0 and np.all(np.where(mapa[i-1:i+2,j] == [1,0,1], True, False)):
                mapa[i-1:i+2, j] = [1,1,1]
            elif mapa[i,j] == 0 and np.all(np.where(mapa[i,j-1:j+2] == [1,0,1], True, False)):
                mapa[i, j-1:j+2] = [1,1,1]
    
    if mapa.shape[0] < mapa.shape[1]:
        to_apend = max(mapa.shape) - mapa.shape[0]
        for _ in range(to_apend):
            mapa = np.insert(mapa, mapa.shape[0], 0, axis=0)
    else:
        to_apend = max(mapa.shape) - mapa.shape[1]
        for _ in range(to_apend):
            mapa = np.insert(mapa, mapa.shape[1], 0, axis=1)
    
    rows = max(mapa.shape)
    cols = max(mapa.shape)
    
    width = rows * 5
    heigh = cols * 5
    
    gap = width // rows
    
    
    win = pygame.display.set_mode((heigh, width))
    pygame.display.set_caption("A* Path Finding Algorithm")
    
    red =   (255, 0, 0)
    green = (0, 255, 0)
    blue =  (0, 0, 255)
    yellow =(255,255,0)
    white = (255,255,255)
    black = (0, 0, 0)
    purple = (128, 0, 128)
    orange = (255, 165, 0)
    grey = (128, 128, 128)
    turquoise = (64, 224, 208)
    
    class Spot:
        def __init__(self, row, col, width, total_rows):
            self.row = row
            self.col = col
            self.x = row * width
            self.y = col * width
            self.color = white
            self.neighbors = []
            self.width = width
            self.heigh = heigh
            self.total_rows = total_rows
            
        def get_pos(self):
            return self.row, self.col
        
        def is_closed(self):
            return self.color == red
        def is_open(self):
            return self.color == green
        def is_barrier(self):
            return self.color == black
        def is_start(self):
            return self.color == yellow
        def is_end(self):
            return self.color == purple
        
        def reset(self):
            self.color = white 
        def make_start(self):
            self.color = orange
        def make_close(self):
            self.color = red
        def make_open(self):
            self.color = green
        def make_barrier(self):
            self.color = black
        def make_end(self):
            self.color = turquoise
        def make_path(self):
            self.color = purple
            
        def draw(self, win):
            pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.heigh))
            
        def update_neighbors(self, grid):
            self.neighbors = []
            if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
                self.neighbors.append(grid[self.row + 1][self.col])
                
            if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
                self.neighbors.append(grid[self.row - 1][self.col])
                
            if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
                self.neighbors.append(grid[self.row][self.col + 1])
                
            if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
                self.neighbors.append(grid[self.row][self.col - 1])
        def __it__(self, other):
            return False
    
    def h(p1, p2):
        x1, y1 = p1
        x2, y2 = p2
        return abs(x2 - x1) + abs(y2 - y1)
    
    def reconstructed_path(came_from, current, draw):
        """
        Aqui está la funcion que necesito.
        current.make_path() es el cambio de color de la celda.
        Introducir en tupla, invertir orden y mandar orden al robot a recorrerla.
        """
        path = []
        while current in came_from:
            current = came_from[current]
            current.make_path()
            path.append([current.row, current.col])
            #draw()
        return np.flip(path)
        
    
    def algorithm(draw, grid, start, end):
        count = 0
        open_set = PriorityQueue()
        open_set.put((0, count, start))
        came_from = {}
        g_score = {spot: float("inf") for row in grid for spot in row}
        g_score[start] = 0
        f_score = {spot: float("inf") for row in grid for spot in row}
        f_score[start] = h(start.get_pos(), end.get_pos())
        
        open_set_hash = {start}
        
        while not open_set.empty():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                
            current = open_set.get()[2]
            open_set_hash.remove(current)
            
            
            if current == start:
                start.make_start()
                
            if current == end:
                path = reconstructed_path(came_from, end, draw)
                print("Shape: ", path.shape)
                path = np.insert(path, path.shape[0], [end.col, end.row], axis=0)
                end.make_end()
                
                return True, path
            
            for neighbor in current.neighbors:
                temp_g_score = g_score[current] + 1
                
                if temp_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g_score
                    f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                    
                    if neighbor not in open_set_hash:
                        count += 1
                        open_set.put((f_score[neighbor], count, neighbor))
                        open_set_hash.add(neighbor)
                        neighbor.make_open()
            #draw()
            
            if current != start: current.make_close()
            
        return False, []
                        
        
    
    def make_grid(rows, cols, width):
        grid = []
        gap = width // rows
        
        
        for i in range(0, rows):
            grid.append([])
            for j in range(0, cols):
                spot = Spot(i, j, gap, rows)
                grid[i].append(spot)
        return grid
    
    def draw_grid(win, rows, cols, width):
        for i in range(0, cols):
            pygame.draw.line(win, grey, (0,i * gap), (width, i * gap))
            for j in range(0, rows):
                pygame.draw.line(win, grey, (j * gap, 0), (j * gap, heigh))
    
    def draw(win, grid, rows, cols, width):
        win.fill(white)
    
        for row in grid:
            for spot in row:
                spot.draw(win)
    
        draw_grid(win, rows, cols, width)
        pygame.display.update()
    
    def get_clicked_pos(pos, rows, width):
        y, x = pos
        row = y // gap
        col = x // gap
        return row, col
    
    def load_map(grid):
        print(mapa.shape)
        print(len(grid))
        print(len(grid[0]))
        print("rows: ",rows)
        print("cols: ", cols)
        for i,row in enumerate(grid):
            for j,spot in enumerate(row):
                celda = mapa[j,i]
                if celda == 1:
                    spot.color = black
                    
    def path_a_robot(path):
        print(path)
        pass
        
    
    def main(win, width):
        grid = make_grid(rows, cols, width)
        load_map(grid)
        
        start = None #grid[start_i][start_j]
        #start.make_start()
        end = None
        
        run = True
        started = False
        
        while run:
            
            draw(win, grid, rows, cols, width)
            for event in pygame.event.get():
                
                if event.type == pygame.QUIT: run = False
                if started: continue
                if pygame.mouse.get_pressed()[0]: # click izquierdo
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, rows, width)
                    spot = grid[row][col]
                    if not start and spot != end:
                        start = spot
                        start.make_start()
                    elif not end and spot != start:
                        end = spot
                        end.make_end()
                    elif spot != end and spot != start:
                        spot.make_barrier()
                        
                elif pygame.mouse.get_pressed()[2]: # click derecho
                    pos = pygame.mouse.get_pos()
                    row, col = get_clicked_pos(pos, rows, width)
                    spot = grid[row][col]
                    spot.reset()
                    if spot == start: start = None
                    elif spot == end: end = None
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE and not started:
                        for row in grid:
                            for spot in row:
                                spot.update_neighbors(grid)
                        done, path = algorithm(lambda: draw(win, grid, rows, cols, width), grid, start, end)
                        print(done)
                        if done: start.make_start()
        
        path_a_robot(path)
        pygame.quit()
        return path
    path = main(win, width)
    return path