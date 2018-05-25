#!/usr/bin/python3

FILE_PATH = "~/p_bld/PathPlanning"

import os, sys
import numpy as np
import xml.etree.ElementTree as ET
from skimage.io import imsave
import skimage.draw as draw

names = [
        "BaldursGate",
        "Mazes1",
        "Mazes2",
        "Mazes4",
        "Moscow",
        "Random20",
        "Random30",
        "Rooms16",
        "Starcraft",
        "Warcraft",
        ]


def run_test(name, alg, opts):
    task = ET.parse("tasks/" + name + ".xml")
    alg_tag = task.getroot().find('algorithm')
    alg_tag.find('searchtype').text = alg
    for option, opt_level in zip(['allowdiagonal', 'cutcorners', 'allowsqueeze'], [3, 2, 1]):
        if opts < opt_level:
            alg_tag.find(option).text = 'true'
        else:
            alg_tag.find(option).text = 'false'
    alg_tag.find('metrictype').text = 'euclidean'
    opt_tag = task.getroot().find('options')
    opt_tag.find('logpath').text = "logs"
    opt_tag.find('logfilename').text = name + "_log.xml"
    task.write("input.xml")
    cmd = FILE_PATH + " input.xml >/dev/null"
    os.system(cmd)


def draw_map(log):
    map_el = log.getroot().find('map')
    x_len = int(map_el.find('width').text)
    y_len = int(map_el.find('height').text)
    img = []
    rows = map_el.find('grid').findall('row')
    for row in rows:
        row = list(map(int, row.text.split()))
        row = np.array([(i, i, i) for i in row])
        img.append(250 * (1 - row))
    return np.array(img)


def draw_map_with_path(log):
    root = log.getroot()
    map_pic = draw_map(log)
    path = root.find('log').find('lplevel').findall('node')
    red_cl = np.array([255, 0, 0])
    for node in path:
        x = int(node.attrib['x'])
        y = int(node.attrib['y'])
        map_pic[y][x] = red_cl
    return map_pic


def run_tests(alg, opts, draw_pic = True):
    print("Running", alg, opts)
    time, length = [], []
    for name in names:
        run_test(name, alg, opts)
        try:
            log = ET.parse('logs/' + name + '_log.xml')
        except:
            print("Log wasn't generated!")
            break
        summary = log.getroot().find('log').find('summary').attrib
        time.append(float(summary['time']))
        length.append(float(summary['length']))
        if draw_pic:
            img = draw_map_with_path(log)
            imsave("pics/" + name + "_" + str(opts) + "_" + alg + ".png", img)
    return time, length


def compare(v_1, v_2):
    assert(len(v_1) == len(v_2))
    assert(len(v_1) == len(names))
    v_3 = [i / j for i, j in zip(v_1, v_2)]
    print(v_3)
    print("Mean:", sum(v_3) / len(v_3))


os.system("rm pics/*")
os.system("rm logs/*")

time_res = dict()
length_res = dict()
opt_num = {"dijkstra": 4, "astar" : 4, "jp_search" : 2, "theta" : 1}
for alg in opt_num:
    for opt in range(opt_num[alg]):
        time_res[(alg, opt)], length_res[(alg, opt)] = run_tests(alg, opt)
print()


print("Dijkstra time / Astar time")
for opts in range(4):
    print("Opts =", opts)
    compare(time_res[('dijkstra', opts)], time_res[('astar', opts)])
print()

print("Astar time / Jump point search")
for opts in range(2):
    print("Opts =", opts)
    compare(time_res[('astar', opts)], time_res[('jp_search', opts)])
print()

print("Theta time / Astar time")
compare(time_res[('theta', 0)], time_res[('astar', 0)])
print()

print("Astar path length / Theta path length")
compare(length_res[('astar', 0)], length_res[('theta', 0)])
print()
