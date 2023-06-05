int64 target_pos   #说话人的目标角度
int64 voice_lenth  #语音长度
string keyword     #说话中的关键词（eg：你好，再见）
uint8  speak_flag  #说话标志位——1：说话,0:未说话
uint8  move_flag   #移动标志位——1：移动，0：停止
uint8  wake_flag   #唤醒标志位——1：唤醒，0：停止
string target_status #到达目标点状态
uint8  step        #根据遥控器决定当前行为
