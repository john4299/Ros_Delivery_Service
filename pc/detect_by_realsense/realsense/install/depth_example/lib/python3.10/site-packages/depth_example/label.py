with open("/home/boma/pixel_data.txt", "w") as file:
                for label, (depth_point_in_color[0], depth_point_in_color[1], depth_point_in_color[2]) in depth_info.items():
                    if "down" in label and current_floor > target[0]:
                        print('down 버튼을 누릅니다.')
                    elif "up" in label and current_floor < target[0]:
                        print('up 버튼을 누릅니다.')
                    elif label[0] == 'b' and label[-1] == str(target[0]):
                        print(f'{target[0]}층 버튼을 누릅니다.')
                    elif label[0] == 'f' and label[-1] == str(target[0]):
                        print(f'{target[0]} 층에 도착하였습니다.')
                        current_floor = target[0]
                        continue
                    elif label == str(target):
                        print('택배를 내려 놓습니다.')
                        continue
                    else:
                        continue
                    file.write(f'{label}: {depth_point_in_color[0]:.5f},{depth_point_in_color[1]:.5f},{depth_point_in_color[2]:.5f}')
                    with open('/home/boma/Desktop/test.txt', 'w') as f:
                        f.write('5')