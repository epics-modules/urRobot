const std::string NEW_LINE= "\n"; const std::string QUOTATION = "\""; std::string UR_SCRIPT = "# HEADER_BEGIN" + NEW_LINE  + 
"    global reg_offset_float = # float register offset" + NEW_LINE  + 
"    global reg_offset_int = # int register offset" + NEW_LINE  + 
"    global force_mode_type = 2" + NEW_LINE  + 
"    global selection_vector = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global task_frame = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global wrench = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global limits = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global is_servoing = 0" + NEW_LINE  + 
"    global is_speeding = 0" + NEW_LINE  + 
"    global is_in_forcemode = 0" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global servo_target = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global servo_time = 0.002" + NEW_LINE  + 
"    global servo_lookahead_time = 0.1" + NEW_LINE  + 
"    global servo_gain = 300" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global servoc_target = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global servoc_acceleration = 1.2" + NEW_LINE  + 
"    global servoc_velocity = 0.25" + NEW_LINE  + 
"    global servoc_blend = 0" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global speed_type = 0" + NEW_LINE  + 
"    global speed_target = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global speed_acceleration = 0.5" + NEW_LINE  + 
"    global speed_time = 0.5" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global move_thrd = 0" + NEW_LINE  + 
"    global move_type = 0" + NEW_LINE  + 
"    global move_p = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global move_q = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global move_vel = 1.2" + NEW_LINE  + 
"    global move_acc = 0.25" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global stop_thrd = 0" + NEW_LINE  + 
"    global stop_type = 0 # 0 = stopj, 1 = stopl" + NEW_LINE  + 
"    global stop_dec = 0" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global servo_thrd = 0" + NEW_LINE  + 
"    global servoc_thrd = 0" + NEW_LINE  + 
"    global speed_thrd = 0" + NEW_LINE  + 
"    global force_thrd = 0" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global jog_feature = 0 # 0 - base, 1 - tool, 2 - custom" + NEW_LINE  + 
"    global jog_speed_pose = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global jog_custom_feature = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global jog_thrd = 0" + NEW_LINE  + 
"    global jog_acc = 0.5" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global contact_direction = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"    global contact_step_back = 0" + NEW_LINE  + 
"    global contact_thrd = 0" + NEW_LINE  + 
"    global internal_cmd = 0" + NEW_LINE  + 
"    " + NEW_LINE  + 
"    def read_input_float_reg(register_index):" + NEW_LINE  + 
"        return read_input_float_register(register_index + reg_offset_float)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def read_input_integer_reg(register_index):" + NEW_LINE  + 
"        return read_input_integer_register(register_index + reg_offset_int)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def write_output_float_reg(register_index, value):" + NEW_LINE  + 
"        write_output_float_register(register_index + reg_offset_float, value)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def write_output_integer_reg(register_index, value):" + NEW_LINE  + 
"        write_output_integer_register(register_index + reg_offset_int, value)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def exec_move_path():" + NEW_LINE  + 
"        textmsg("+QUOTATION+"exec_move_path"+QUOTATION+")" + NEW_LINE  + 
"        # inject move path" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"" + NEW_LINE  + 
"    global async_wr_count = 0 # is incremented each time the register is written" + NEW_LINE  + 
"    global async_op_id = 0 # is incremented each time a new async operation is started" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # Write async progress register" + NEW_LINE  + 
"    # This function provides extended progress information in an async progress " + NEW_LINE  + 
"    # status register. The following bits are used" + NEW_LINE  + 
"    #" + NEW_LINE  + 
"    # | Bit 31 | Bit 30 - 24 | Bit 23 | Bit 9 - 22     | Bit 15  | Bit 14 - 0 |" + NEW_LINE  + 
"    # +--------+-------------+--------+----------------+---------+------------+" + NEW_LINE  + 
"    # | rsv    | async_op_id | rsv    | async_wr_count | running | progress   | " + NEW_LINE  + 
"    #" + NEW_LINE  + 
"    # running: 1 - async op. running, 0 - async op. finished" + NEW_LINE  + 
"    # progress: progress of running async. operation" + NEW_LINE  + 
"    #" + NEW_LINE  + 
"    def write_async_progress_register(value):" + NEW_LINE  + 
"        # we use 7 bits for the change count - that means 127 is our maximum" + NEW_LINE  + 
"        async_wr_count = async_wr_count + 1" + NEW_LINE  + 
"        if async_wr_count > 127:" + NEW_LINE  + 
"            async_wr_count = 0" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        # 0 means new async operation started - we increment the opration id" + NEW_LINE  + 
"        # we use 7 bits for the operation id - that means 127 is our maximum" + NEW_LINE  + 
"        if value == 0:" + NEW_LINE  + 
"            async_op_id = async_op_id + 1" + NEW_LINE  + 
"            if async_op_id > 127:" + NEW_LINE  + 
"                async_op_id = 0" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        reg_value = async_wr_count * 65536 # shift change counter to bits 16 - 23" + NEW_LINE  + 
"        reg_value = reg_value + (async_op_id * 16777216) # shift opration id to bits 24 - 30" + NEW_LINE  + 
"" + NEW_LINE  + 
"        # if an async operation is active, we set the bit 15 (value 32768) and" + NEW_LINE  + 
"        # store the progress value into the lower 15 bit" + NEW_LINE  + 
"        if value >= 0:" + NEW_LINE  + 
"            # ensure that the progress value only uses 15 bit bacause bit 15" + NEW_LINE  + 
"            # is used for indication of runnin state" + NEW_LINE  + 
"            if value > 32767:" + NEW_LINE  + 
"                value = 32767" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            reg_value = reg_value + value + 32768 # store the progress in the lower 16 bits            " + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        textmsg("+QUOTATION+"async_wr_count: "+QUOTATION+", async_wr_count)" + NEW_LINE  + 
"        textmsg("+QUOTATION+"async_op_id: "+QUOTATION+", async_op_id)" + NEW_LINE  + 
"        write_output_integer_reg(2, reg_value)" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"" + NEW_LINE  + 
"      def signal_async_progress(value):" + NEW_LINE  + 
"        write_async_progress_register(value)" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"  " + NEW_LINE  + 
"      def signal_async_operation_started():" + NEW_LINE  + 
"        write_async_progress_register(0) # 0 indicates operation started" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"  " + NEW_LINE  + 
"      def signal_async_operation_finished():" + NEW_LINE  + 
"        write_async_progress_register(-1) # negative values indicate operation finished" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"  " + NEW_LINE  + 
"      def reset_async_progress():" + NEW_LINE  + 
"        signal_async_operation_finished()" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # asynchronous stop thread" + NEW_LINE  + 
"    # stopping a move with a low deceleration may take quite some time, so" + NEW_LINE  + 
"    # async execution may prevent blocking the caller" + NEW_LINE  + 
"    thread stop_thread():" + NEW_LINE  + 
"      textmsg("+QUOTATION+"stop_thread started"+QUOTATION+")" + NEW_LINE  + 
"      signal_async_operation_started()" + NEW_LINE  + 
"      while (True):" + NEW_LINE  + 
"          if stop_type == 0:" + NEW_LINE  + 
"              stopj(stop_dec)" + NEW_LINE  + 
"          elif stop_type == 1:" + NEW_LINE  + 
"              stopl(stop_dec)" + NEW_LINE  + 
"          end" + NEW_LINE  + 
"          enter_critical" + NEW_LINE  + 
"          stop_thrd = 0" + NEW_LINE  + 
"          textmsg("+QUOTATION+"stop_thread finished"+QUOTATION+")" + NEW_LINE  + 
"          exit_critical" + NEW_LINE  + 
"          break" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"      signal_async_operation_finished()" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread move_thread():" + NEW_LINE  + 
"        textmsg("+QUOTATION+"move_thread started"+QUOTATION+")" + NEW_LINE  + 
"        signal_async_operation_started()" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            if move_type == 0 or move_type == 1:" + NEW_LINE  + 
"                movej(move_q, a=move_acc, v=move_vel)" + NEW_LINE  + 
"            elif move_type == 2:" + NEW_LINE  + 
"                movel(move_p, a=move_acc, v=move_vel)" + NEW_LINE  + 
"            elif move_type == 3:" + NEW_LINE  + 
"                movel(move_q, a=move_acc, v=move_vel)" + NEW_LINE  + 
"            elif move_type == 4:" + NEW_LINE  + 
"                exec_move_path()" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            move_thrd = 0" + NEW_LINE  + 
"            textmsg("+QUOTATION+"move_thread finished"+QUOTATION+")" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            break" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        signal_async_operation_finished()" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def stop_async_move():" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        if move_thrd != 0:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"stopping async movement - killing move_thrd"+QUOTATION+")" + NEW_LINE  + 
"            kill move_thrd" + NEW_LINE  + 
"            move_thrd = 0" + NEW_LINE  + 
"            # If the move thread is killed, it cannot trigger the " + NEW_LINE  + 
"            # signal_async_operation_finished() signal - so we do it here" + NEW_LINE  + 
"            signal_async_operation_finished()" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # Execute an sync or async stopl or stopj command." + NEW_LINE  + 
"    def exec_stopl_stopj(cmd, message):" + NEW_LINE  + 
"      deceleration_rate = read_input_float_reg(0)" + NEW_LINE  + 
"      async = read_input_integer_reg(1)" + NEW_LINE  + 
"      textmsg(message, async)" + NEW_LINE  + 
"      # if a stop thread is already running, we kill it so that we can restart it" + NEW_LINE  + 
"      # with the new parameters in the code below" + NEW_LINE  + 
"      stop_move = True" + NEW_LINE  + 
"      enter_critical" + NEW_LINE  + 
"      if stop_thrd != 0:" + NEW_LINE  + 
"          stop_move = False # if a stop thread is running then someone already called async stop before" + NEW_LINE  + 
"          textmsg("+QUOTATION+"killing stop_thrd"+QUOTATION+")" + NEW_LINE  + 
"          kill stop_thrd" + NEW_LINE  + 
"          stop_thrd = 0" + NEW_LINE  + 
"          signal_async_operation_finished()" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"      exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"      # We only need to stop robot movement, if it has not been stopped by a" + NEW_LINE  + 
"      # previous running async stop command" + NEW_LINE  + 
"      if stop_move == True:" + NEW_LINE  + 
"        stop_async_move()" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"" + NEW_LINE  + 
"      # If this is an async stop, then start the stop thread now. If not, then" + NEW_LINE  + 
"      # we stop synchronously" + NEW_LINE  + 
"      if async == 1:" + NEW_LINE  + 
"          enter_critical" + NEW_LINE  + 
"          stop_type = 1" + NEW_LINE  + 
"          stop_dec = deceleration_rate" + NEW_LINE  + 
"          exit_critical" + NEW_LINE  + 
"          stop_thrd = run stop_thread()" + NEW_LINE  + 
"      else:" + NEW_LINE  + 
"          if stop_type == 0:" + NEW_LINE  + 
"            stopj(deceleration_rate)" + NEW_LINE  + 
"          elif stop_type == 1:" + NEW_LINE  + 
"            stopl(deceleration_rate)" + NEW_LINE  + 
"          end" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread force_thread():" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            force_mode(task_frame, selection_vector, wrench, force_mode_type, limits)" + NEW_LINE  + 
"            sync()" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread speed_thread():" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            type = speed_type" + NEW_LINE  + 
"            target = speed_target" + NEW_LINE  + 
"            acceleration = speed_acceleration" + NEW_LINE  + 
"            time = speed_time" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            if type == 0:" + NEW_LINE  + 
"                if time > 0:" + NEW_LINE  + 
"                    speedl(target, a=acceleration, t=time)" + NEW_LINE  + 
"                else:" + NEW_LINE  + 
"                    speedl(target, a=acceleration)" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                if time > 0:" + NEW_LINE  + 
"                    speedj(target, a=acceleration, t=time)" + NEW_LINE  + 
"                else:" + NEW_LINE  + 
"                    speedj(target, a=acceleration)" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread servo_thread():" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            q = servo_target" + NEW_LINE  + 
"            dt = servo_time" + NEW_LINE  + 
"            lh_time = servo_lookahead_time" + NEW_LINE  + 
"            g = servo_gain" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            servoj(q, t=dt, lookahead_time=lh_time, gain=g)" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread servoc_thread():" + NEW_LINE  + 
"        while (True):" + NEW_LINE  + 
"            servoc(servoc_target, a=servoc_acceleration, v=servoc_velocity, r=servoc_blend)" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    thread contact_thread():" + NEW_LINE  + 
"      textmsg("+QUOTATION+"contact_thread started"+QUOTATION+")" + NEW_LINE  + 
"      enter_critical" + NEW_LINE  + 
"      contact_step_back = 0" + NEW_LINE  + 
"      dir = contact_direction" + NEW_LINE  + 
"      exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"      textmsg("+QUOTATION+"direction:"+QUOTATION+", dir)" + NEW_LINE  + 
"      use_speed_vector = pose_is_null(dir)" + NEW_LINE  + 
"      if use_speed_vector:" + NEW_LINE  + 
"          textmsg("+QUOTATION+"Using speed vector"+QUOTATION+")" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"" + NEW_LINE  + 
"      step_back = 1" + NEW_LINE  + 
"      # First we wait until there is not contact" + NEW_LINE  + 
"$5.4  while step_back > 0:" + NEW_LINE  + 
"$5.4      if use_speed_vector:" + NEW_LINE  + 
"$5.4          step_back = tool_contact(direction=get_target_tcp_speed())" + NEW_LINE  + 
"$5.4      else:" + NEW_LINE  + 
"$5.4          step_back = tool_contact(direction=dir)" + NEW_LINE  + 
"$5.4      end" + NEW_LINE  + 
"$5.4      sync()" + NEW_LINE  + 
"$5.4  end" + NEW_LINE  + 
"" + NEW_LINE  + 
"      # Now we can wait for a contact" + NEW_LINE  + 
"$5.4  while step_back == 0:" + NEW_LINE  + 
"$5.4      if use_speed_vector:" + NEW_LINE  + 
"$5.4          step_back = tool_contact(direction=get_target_tcp_speed())" + NEW_LINE  + 
"$5.4      else:" + NEW_LINE  + 
"$5.4          step_back = tool_contact(direction=dir)" + NEW_LINE  + 
"$5.4      end" + NEW_LINE  + 
"$5.4      sync()" + NEW_LINE  + 
"$5.4  end" + NEW_LINE  + 
"      textmsg("+QUOTATION+"step_back:"+QUOTATION+", step_back)" + NEW_LINE  + 
"      textmsg("+QUOTATION+"target_tcp_speed: "+QUOTATION+", get_target_tcp_speed())" + NEW_LINE  + 
"      enter_critical" + NEW_LINE  + 
"      contact_thrd = 0" + NEW_LINE  + 
"      contact_step_back = step_back" + NEW_LINE  + 
"      internal_cmd = 254 # delegate processing of contact detection to main thread" + NEW_LINE  + 
"      exit_critical" + NEW_LINE  + 
"      textmsg("+QUOTATION+"contact_thread finished"+QUOTATION+")" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def q_from_input_float_registers(register_index):" + NEW_LINE  + 
"      q = [0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"      q[0] = read_input_float_reg(register_index + 0)" + NEW_LINE  + 
"      q[1] = read_input_float_reg(register_index + 1)" + NEW_LINE  + 
"      q[2] = read_input_float_reg(register_index + 2)" + NEW_LINE  + 
"      q[3] = read_input_float_reg(register_index + 3)" + NEW_LINE  + 
"      q[4] = read_input_float_reg(register_index + 4)" + NEW_LINE  + 
"      q[5] = read_input_float_reg(register_index + 5)" + NEW_LINE  + 
"      return q" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def pose_from_input_float_registers(register_index):" + NEW_LINE  + 
"      pose = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"      pose[0] = read_input_float_reg(register_index + 0)" + NEW_LINE  + 
"      pose[1] = read_input_float_reg(register_index + 1)" + NEW_LINE  + 
"      pose[2] = read_input_float_reg(register_index + 2)" + NEW_LINE  + 
"      pose[3] = read_input_float_reg(register_index + 3)" + NEW_LINE  + 
"      pose[4] = read_input_float_reg(register_index + 4)" + NEW_LINE  + 
"      pose[5] = read_input_float_reg(register_index + 5)" + NEW_LINE  + 
"      return pose" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def pose_to_output_float_registers(register_index, pose):" + NEW_LINE  + 
"      write_output_float_reg(register_index + 0, pose[0])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 1, pose[1])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 2, pose[2])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 3, pose[3])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 4, pose[4])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 5, pose[5])" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def q_to_output_float_registers(register_index, q):" + NEW_LINE  + 
"      write_output_float_reg(register_index + 0, q[0])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 1, q[1])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 2, q[2])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 3, q[3])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 4, q[4])" + NEW_LINE  + 
"      write_output_float_reg(register_index + 5, q[5])" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # Returs a pose that contains only the translation part of a given pose" + NEW_LINE  + 
"    def get_pose_translation(pose):" + NEW_LINE  + 
"      return p[pose[0], pose[1], pose[2], 0, 0, 0]" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"  " + NEW_LINE  + 
"    # Returns a pose that contains only the rotation part of a given pose" + NEW_LINE  + 
"    def get_pose_rotation(pose):" + NEW_LINE  + 
"      return p[0, 0, 0, pose[3], pose[4], pose[5]]" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # scales the given pose by the given factor" + NEW_LINE  + 
"    def scale_pose(p, factor):" + NEW_LINE  + 
"      p[0] = p[0] * factor" + NEW_LINE  + 
"      p[1] = p[1] * factor" + NEW_LINE  + 
"      p[2] = p[2] * factor" + NEW_LINE  + 
"      p[3] = p[3] * factor" + NEW_LINE  + 
"      p[4] = p[4] * factor" + NEW_LINE  + 
"      p[5] = p[5] * factor" + NEW_LINE  + 
"      return p" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def pose_is_null(p):" + NEW_LINE  + 
"      sum = p[0] + p[1] + p[2] + p[3] + p[4] + p[5]" + NEW_LINE  + 
"      return (sum < 0.0000000001)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    # convert tool_speed expressed in the frame_wrt_base in the robot base frame" + NEW_LINE  + 
"    # examples :" + NEW_LINE  + 
"    # speed_wrt_base([0.020, 0.020,0,0,0,0], plane_1)" + NEW_LINE  + 
"    # speedl(get_speed_wrt_base([0,0,0.02,0,0,1.57], get_actual_tcp_pose()),a=1,t=0.5,aRot=4))" + NEW_LINE  + 
"  " + NEW_LINE  + 
"    #### input arguments" + NEW_LINE  + 
"    # tool_speed : [Tx, Ty, Tz, Rx, Ry, Rz] spatial vector list - Txyz [m/s]), Rxyz [rad/s]" + NEW_LINE  + 
"    # frame_wrt_base : pose" + NEW_LINE  + 
"  " + NEW_LINE  + 
"    #### output arguments" + NEW_LINE  + 
"    # speed_wrt_base : [Tx, Ty, Tz, Rx, Ry, Rz] spatial vector list - Txyz [m/s]), Rxyz [rad/s]" + NEW_LINE  + 
"    def get_speed_wrt_base(tool_speed, frame_wrt_base):    " + NEW_LINE  + 
"      # Translationnal speed vector calculus" + NEW_LINE  + 
"      T_wrt_frame=p[tool_speed[0],tool_speed[1],tool_speed[2],0,0,0]" + NEW_LINE  + 
"      T_wrt_base_raw= pose_trans (frame_wrt_base, T_wrt_frame)" + NEW_LINE  + 
"      T_wrt_base=pose_sub(T_wrt_base_raw,frame_wrt_base)" + NEW_LINE  + 
"    " + NEW_LINE  + 
"      # Rotationnal speed vector calculus" + NEW_LINE  + 
"      R_wrt_frame=p[tool_speed[3],tool_speed[4],tool_speed[5],0,0,0]" + NEW_LINE  + 
"      R_wrt_frame_abs=norm([R_wrt_frame[0],R_wrt_frame[1],R_wrt_frame[2]])" + NEW_LINE  + 
"      if R_wrt_frame_abs != 0:  # test if zero vector" + NEW_LINE  + 
"        R_wrt_base_raw=pose_trans(frame_wrt_base, R_wrt_frame)" + NEW_LINE  + 
"        R_wrt_base=pose_sub(R_wrt_base_raw, frame_wrt_base)" + NEW_LINE  + 
"      else:" + NEW_LINE  + 
"        R_wrt_base=p[0,0,0,0,0,0]" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"    " + NEW_LINE  + 
"      # Concatenate T and R" + NEW_LINE  + 
"      speed_wrt_base_list=[T_wrt_base[0],T_wrt_base[1],T_wrt_base[2],R_wrt_base[0],R_wrt_base[1],R_wrt_base[2]]" + NEW_LINE  + 
"      return speed_wrt_base_list   " + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    " + NEW_LINE  + 
"    # The jog thread provides jogging functionality similar to the one you" + NEW_LINE  + 
"    # find in the teach pendant." + NEW_LINE  + 
"    thread jog_speed_thread():" + NEW_LINE  + 
"      textmsg("+QUOTATION+"jog_speed_thread started"+QUOTATION+")" + NEW_LINE  + 
"      enter_critical" + NEW_LINE  + 
"      feature = jog_feature" + NEW_LINE  + 
"      exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"      # Initialize target_pose depending on feature" + NEW_LINE  + 
"      if feature == 0: # base feature" + NEW_LINE  + 
"        frame_wrt_base = p[0, 0, 0, 0, 0, 0]" + NEW_LINE  + 
"      elif feature == 1: # tool feature" + NEW_LINE  + 
"        frame_wrt_base = get_actual_tcp_pose()" + NEW_LINE  + 
"      elif feature == 2: # custom feature" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        frame_wrt_base = jog_custom_feature" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"" + NEW_LINE  + 
"      while (True):" + NEW_LINE  + 
"        enter_critical" + NEW_LINE  + 
"        speed_pose = jog_speed_pose" + NEW_LINE  + 
"        exit_critical" + NEW_LINE  + 
"        speed_wrt_base = get_speed_wrt_base(speed_pose, frame_wrt_base)" + NEW_LINE  + 
"        speedl(speed_wrt_base, jog_acc)" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def signal_ready():" + NEW_LINE  + 
"        write_output_integer_reg(0, 1)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def signal_done_with_cmd():" + NEW_LINE  + 
"        write_output_integer_reg(0, 2)" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def rtde_cmd():" + NEW_LINE  + 
"      cmd = read_input_integer_reg(0)" + NEW_LINE  + 
"      # only if command is 0 - that means we are idle, we process internal" + NEW_LINE  + 
"      # commands" + NEW_LINE  + 
"      if cmd == 0:" + NEW_LINE  + 
"          enter_critical" + NEW_LINE  + 
"          cmd = internal_cmd" + NEW_LINE  + 
"          internal_cmd = 0" + NEW_LINE  + 
"          exit_critical" + NEW_LINE  + 
"      end" + NEW_LINE  + 
"      return cmd" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"    def process_cmd(cmd):" + NEW_LINE  + 
"        if cmd == 1:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movej"+QUOTATION+")" + NEW_LINE  + 
"            q = q_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"            async = read_input_integer_reg(1)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            stop_async_move()" + NEW_LINE  + 
"            if async == 1:" + NEW_LINE  + 
"                enter_critical" + NEW_LINE  + 
"                move_type = 0" + NEW_LINE  + 
"                move_q = q" + NEW_LINE  + 
"                move_acc = acceleration" + NEW_LINE  + 
"                move_vel = velocity" + NEW_LINE  + 
"                exit_critical" + NEW_LINE  + 
"                move_thrd = run move_thread()" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                movej(q, a=acceleration, v=velocity)" + NEW_LINE  + 
"                textmsg("+QUOTATION+"movej done"+QUOTATION+")" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 2:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movej_ik"+QUOTATION+")" + NEW_LINE  + 
"            pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"            async = read_input_integer_reg(1)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target pose:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose)" + NEW_LINE  + 
"            q = get_inverse_kin(pose)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            stop_async_move()" + NEW_LINE  + 
"            if async == 1:" + NEW_LINE  + 
"                enter_critical" + NEW_LINE  + 
"                move_type = 1" + NEW_LINE  + 
"                move_q = q" + NEW_LINE  + 
"                move_acc = acceleration" + NEW_LINE  + 
"                move_vel = velocity" + NEW_LINE  + 
"                exit_critical" + NEW_LINE  + 
"                move_thrd = run move_thread()" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                movej(q, a=acceleration, v=velocity)" + NEW_LINE  + 
"                textmsg("+QUOTATION+"movej_ik done"+QUOTATION+")" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 3:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movel"+QUOTATION+")" + NEW_LINE  + 
"            pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"            async = read_input_integer_reg(1)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target pose:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(pose)" + NEW_LINE  + 
"            stop_async_move()" + NEW_LINE  + 
"            if async == 1:" + NEW_LINE  + 
"                enter_critical" + NEW_LINE  + 
"                move_type = 2" + NEW_LINE  + 
"                move_p = pose" + NEW_LINE  + 
"                move_acc = acceleration" + NEW_LINE  + 
"                move_vel = velocity" + NEW_LINE  + 
"                exit_critical" + NEW_LINE  + 
"                move_thrd = run move_thread()" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                movel(pose, a=acceleration, v=velocity)" + NEW_LINE  + 
"                textmsg("+QUOTATION+"movel done"+QUOTATION+")" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 4:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"movel_fk"+QUOTATION+")" + NEW_LINE  + 
"            q = q_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"            async = read_input_integer_reg(1)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"Target q:"+QUOTATION+")" + NEW_LINE  + 
"            textmsg(q)" + NEW_LINE  + 
"            stop_async_move()" + NEW_LINE  + 
"            if async == 1:" + NEW_LINE  + 
"                enter_critical" + NEW_LINE  + 
"                move_type = 3" + NEW_LINE  + 
"                move_q = q" + NEW_LINE  + 
"                move_acc = acceleration" + NEW_LINE  + 
"                move_vel = velocity" + NEW_LINE  + 
"                exit_critical" + NEW_LINE  + 
"                move_thrd = run move_thread()" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                movel(q, a=acceleration, v=velocity)" + NEW_LINE  + 
"                textmsg("+QUOTATION+"movel_fk done"+QUOTATION+")" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 6:" + NEW_LINE  + 
"            # force_mode" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            force_mode_type = read_input_integer_reg(1)" + NEW_LINE  + 
"            selection_vector[0] = read_input_integer_reg(2)" + NEW_LINE  + 
"            selection_vector[1] = read_input_integer_reg(3)" + NEW_LINE  + 
"            selection_vector[2] = read_input_integer_reg(4)" + NEW_LINE  + 
"            selection_vector[3] = read_input_integer_reg(5)" + NEW_LINE  + 
"            selection_vector[4] = read_input_integer_reg(6)" + NEW_LINE  + 
"            selection_vector[5] = read_input_integer_reg(7)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            task_frame = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            wrench = q_from_input_float_registers(6)" + NEW_LINE  + 
"            limits = q_from_input_float_registers(12)" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_in_forcemode == 0:" + NEW_LINE  + 
"                is_in_forcemode = 1" + NEW_LINE  + 
"                if force_thrd == 0:" + NEW_LINE  + 
"                    global force_thrd = run force_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 7:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_stop"+QUOTATION+")" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            is_in_forcemode = 0" + NEW_LINE  + 
"            kill force_thrd" + NEW_LINE  + 
"            force_thrd = 0" + NEW_LINE  + 
"            end_force_mode()" + NEW_LINE  + 
"            stopl(10)" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode stopped"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 8:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"zero_ftsensor"+QUOTATION+")" + NEW_LINE  + 
"            zero_ftsensor()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"ftsensor zeroed"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 9:" + NEW_LINE  + 
"            # speedJ" + NEW_LINE  + 
"            qd = q_from_input_float_registers(0)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            speed_type = 1" + NEW_LINE  + 
"            speed_acceleration = read_input_float_reg(6)" + NEW_LINE  + 
"            speed_time = read_input_float_reg(7)" + NEW_LINE  + 
"            speed_target = qd" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_speeding == 0:" + NEW_LINE  + 
"                enter_critical" + NEW_LINE  + 
"                is_speeding = 1" + NEW_LINE  + 
"                exit_critical" + NEW_LINE  + 
"                if speed_thrd == 0:" + NEW_LINE  + 
"                    global speed_thrd = run speed_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 10:" + NEW_LINE  + 
"            # speedL" + NEW_LINE  + 
"            xd = q_from_input_float_registers(0)" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            speed_type = 0" + NEW_LINE  + 
"            speed_acceleration = read_input_float_reg(6)" + NEW_LINE  + 
"            speed_time = read_input_float_reg(7)" + NEW_LINE  + 
"            speed_target = xd" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_speeding == 0:" + NEW_LINE  + 
"                is_speeding = 1" + NEW_LINE  + 
"                if speed_thrd == 0:" + NEW_LINE  + 
"                    global speed_thrd = run speed_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 11:" + NEW_LINE  + 
"            # servoJ" + NEW_LINE  + 
"            q = q_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            servo_target = q" + NEW_LINE  + 
"            servo_time = read_input_float_reg(8)" + NEW_LINE  + 
"            servo_lookahead_time = read_input_float_reg(9)" + NEW_LINE  + 
"            servo_gain = read_input_float_reg(10)" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_servoing == 0:" + NEW_LINE  + 
"                is_servoing = 1" + NEW_LINE  + 
"                if servo_thrd == 0:" + NEW_LINE  + 
"                    global servo_thrd = run servo_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        elif cmd == 12:" + NEW_LINE  + 
"            # servoC" + NEW_LINE  + 
"            pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"            blend = read_input_float_reg(8)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            servoc_target = pose" + NEW_LINE  + 
"            servoc_acceleration = acceleration" + NEW_LINE  + 
"            servoc_velocity = velocity" + NEW_LINE  + 
"            servoc_blend = blend" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_servoing == 0:" + NEW_LINE  + 
"                is_servoing = 1" + NEW_LINE  + 
"                if servoc_thrd == 0:" + NEW_LINE  + 
"                    global servoc_thrd = run servoc_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        elif cmd == 15:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speed_stop"+QUOTATION+")" + NEW_LINE  + 
"            deceleration_rate = read_input_float_reg(0)" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            is_speeding = 0" + NEW_LINE  + 
"            kill speed_thrd" + NEW_LINE  + 
"            speed_thrd = 0" + NEW_LINE  + 
"            if speed_type == 0:" + NEW_LINE  + 
"                stopl(deceleration_rate)" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                stopj(deceleration_rate)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            textmsg("+QUOTATION+"speed_stop done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 16:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servo_stop"+QUOTATION+")" + NEW_LINE  + 
"            deceleration_rate = read_input_float_reg(0)" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            is_servoing = 0" + NEW_LINE  + 
"            kill servo_thrd" + NEW_LINE  + 
"            kill servoc_thrd" + NEW_LINE  + 
"            servo_thrd = 0" + NEW_LINE  + 
"            servoc_thrd = 0" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"            stopl(deceleration_rate)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"servo_stop done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 17:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_payload"+QUOTATION+")" + NEW_LINE  + 
"            mass = read_input_float_reg(0)" + NEW_LINE  + 
"            cog_x = read_input_float_reg(1)" + NEW_LINE  + 
"            cog_y = read_input_float_reg(2)" + NEW_LINE  + 
"            cog_z = read_input_float_reg(3)" + NEW_LINE  + 
"            cog = [cog_x, cog_y, cog_z]" + NEW_LINE  + 
"$5.0        if cog_x == 0 and cog_y == 0 and cog_z == 0:" + NEW_LINE  + 
"$5.0            set_payload(mass, get_target_payload_cog())" + NEW_LINE  + 
"$5.0        else:" + NEW_LINE  + 
"                set_payload(mass, cog)" + NEW_LINE  + 
"$5.0        end" + NEW_LINE  + 
"$5.0        textmsg("+QUOTATION+"active payload:"+QUOTATION+")" + NEW_LINE  + 
"$5.0        textmsg(get_target_payload())" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_payload done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 18:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"teach_mode"+QUOTATION+")" + NEW_LINE  + 
"            teach_mode()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"teach_mode done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 19:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"end_teach_mode"+QUOTATION+")" + NEW_LINE  + 
"            end_teach_mode()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"end_teach_mode done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 20:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_damping"+QUOTATION+")" + NEW_LINE  + 
"            damping = read_input_float_reg(0)" + NEW_LINE  + 
"$3.5        force_mode_set_damping(damping)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_damping done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 21:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_gain_scaling"+QUOTATION+")" + NEW_LINE  + 
"            scaling = read_input_float_reg(0)" + NEW_LINE  + 
"$5.0        force_mode_set_gain_scaling(scaling)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"force_mode_set_gain_scaling done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 24:" + NEW_LINE  + 
"            pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            velocity = read_input_float_reg(6)" + NEW_LINE  + 
"            acceleration = read_input_float_reg(7)" + NEW_LINE  + 
"            q = get_inverse_kin(pose)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            enter_critical" + NEW_LINE  + 
"            servo_target = q" + NEW_LINE  + 
"            servo_time = read_input_float_reg(8)" + NEW_LINE  + 
"            servo_lookahead_time = read_input_float_reg(9)" + NEW_LINE  + 
"            servo_gain = read_input_float_reg(10)" + NEW_LINE  + 
"            exit_critical" + NEW_LINE  + 
"" + NEW_LINE  + 
"            if is_servoing == 0:" + NEW_LINE  + 
"                is_servoing = 1" + NEW_LINE  + 
"                if servo_thrd == 0:" + NEW_LINE  + 
"                    global servo_thrd = run servo_thread()" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"        elif cmd == 25:" + NEW_LINE  + 
"            # tool_contact" + NEW_LINE  + 
"            direction = pose_from_input_float_registers(0)" + NEW_LINE  + 
"$5.4        time_steps = tool_contact(direction)" + NEW_LINE  + 
"$5.4        write_output_integer_reg(1, time_steps)" + NEW_LINE  + 
"        elif cmd == 26:" + NEW_LINE  + 
"            # get_steptime" + NEW_LINE  + 
"$5.4        step_time = get_steptime()" + NEW_LINE  + 
"$5.4        write_output_float_reg(0, step_time)" + NEW_LINE  + 
"        elif cmd == 27:" + NEW_LINE  + 
"            # get_actual_joint_positions_history" + NEW_LINE  + 
"            steps = read_input_integer_reg(1)" + NEW_LINE  + 
"$5.4        joint_positions_history = get_actual_joint_positions_history(steps)" + NEW_LINE  + 
"$5.4        q_to_output_float_registers(0, joint_positions_history)" + NEW_LINE  + 
"        elif cmd == 28:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"get_target_waypoint"+QUOTATION+")" + NEW_LINE  + 
"$5.3        target_waypoint = get_target_waypoint()" + NEW_LINE  + 
"$5.3        pose_to_output_float_registers(0, target_waypoint)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"get_target_waypoint done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 29:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_tcp"+QUOTATION+")" + NEW_LINE  + 
"            pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            set_tcp(pose)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_tcp done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 30:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"get_inverse_kin_args"+QUOTATION+")" + NEW_LINE  + 
"            x = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            qnear = q_from_input_float_registers(6)" + NEW_LINE  + 
"            maxPositionError = read_input_float_reg(12)" + NEW_LINE  + 
"            maxOrientationError = read_input_float_reg(13)" + NEW_LINE  + 
"" + NEW_LINE  + 
"            q = get_inverse_kin(x, qnear, maxPositionError, maxOrientationError)" + NEW_LINE  + 
"            q_to_output_float_registers(0, q)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"get_inverse_kin done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 31:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"protective_stop"+QUOTATION+")" + NEW_LINE  + 
"$3.8        protective_stop()" + NEW_LINE  + 
"            textmsg("+QUOTATION+"protective_stop done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 33:" + NEW_LINE  + 
"            exec_stopl_stopj(cmd, "+QUOTATION+"stopl async="+QUOTATION+")" + NEW_LINE  + 
"            textmsg("+QUOTATION+"stopl done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 34:" + NEW_LINE  + 
"            exec_stopl_stopj(cmd, "+QUOTATION+"stopj async="+QUOTATION+")" + NEW_LINE  + 
"            textmsg("+QUOTATION+"stopj done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 35:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_watchdog"+QUOTATION+")" + NEW_LINE  + 
"            # Setup watchdog for the RTDE communication" + NEW_LINE  + 
"            watchdog_min_frequency = read_input_float_reg(0)" + NEW_LINE  + 
"            if reg_offset_int == 0:" + NEW_LINE  + 
"                rtde_set_watchdog("+QUOTATION+"input_int_register_0"+QUOTATION+", watchdog_min_frequency, "+QUOTATION+"stop"+QUOTATION+")" + NEW_LINE  + 
"            elif reg_offset_int == 24:" + NEW_LINE  + 
"                rtde_set_watchdog("+QUOTATION+"input_int_register_24"+QUOTATION+", watchdog_min_frequency, "+QUOTATION+"stop"+QUOTATION+")" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                rtde_set_watchdog("+QUOTATION+"input_int_register_0"+QUOTATION+", watchdog_min_frequency, "+QUOTATION+"stop"+QUOTATION+")" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            textmsg("+QUOTATION+"set_watchdog done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 36:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"is_pose_within_safety_limits"+QUOTATION+")" + NEW_LINE  + 
"            pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            safe_pose = is_within_safety_limits(pose)" + NEW_LINE  + 
"            if safe_pose == True:" + NEW_LINE  + 
"               write_output_integer_reg(1, 1)" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"               write_output_integer_reg(1, 0)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            textmsg("+QUOTATION+"is_pose_within_safety_limits done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 37:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"is_joints_within_safety_limits"+QUOTATION+")" + NEW_LINE  + 
"            q = q_from_input_float_registers(0)" + NEW_LINE  + 
"            safe_q = is_within_safety_limits(q)" + NEW_LINE  + 
"            if safe_q == True:" + NEW_LINE  + 
"               write_output_integer_reg(1, 1)" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"               write_output_integer_reg(1, 0)" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"            textmsg("+QUOTATION+"is_joints_within_safety_limits done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 38:" + NEW_LINE  + 
"            # get_joint_torques" + NEW_LINE  + 
"            torques = get_joint_torques()" + NEW_LINE  + 
"            q_to_output_float_registers(0, torques)" + NEW_LINE  + 
"        elif cmd == 39:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"pose_trans"+QUOTATION+")" + NEW_LINE  + 
"            p_from = pose_from_input_float_registers(0)" + NEW_LINE  + 
"            p_from_to = pose_from_input_float_registers(6)" + NEW_LINE  + 
"            p = pose_trans(p_from, p_from_to)" + NEW_LINE  + 
"            pose_to_output_float_registers(0, p)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"pose_trans done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 40:" + NEW_LINE  + 
"            textmsg("+QUOTATION+"get_tcp_offset"+QUOTATION+")" + NEW_LINE  + 
"$3.6        tcp_offset = get_tcp_offset()" + NEW_LINE  + 
"$3.6        textmsg(tcp_offset)" + NEW_LINE  + 
"$3.6        pose_to_output_float_registers(0, tcp_offset)" + NEW_LINE  + 
"            textmsg("+QUOTATION+"get_tcp_offset done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 41:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"start_jog"+QUOTATION+")" + NEW_LINE  + 
"              enter_critical" + NEW_LINE  + 
"              jog_speed_pose = pose_from_input_float_registers(0)" + NEW_LINE  + 
"              jog_feature = read_input_float_reg(6)" + NEW_LINE  + 
"              jog_acc = read_input_float_reg(7)" + NEW_LINE  + 
"              jog_custom_feature = pose_from_input_float_registers(8)" + NEW_LINE  + 
"              if jog_thrd == 0:" + NEW_LINE  + 
"                jog_thrd = run jog_speed_thread()" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              exit_critical" + NEW_LINE  + 
"              textmsg("+QUOTATION+"jog_feature: "+QUOTATION+", jog_feature)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"jog_custom_feature: "+QUOTATION+", jog_custom_feature)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"jog_acc: "+QUOTATION+", jog_acc)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"start_jog done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 42:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"stop_jog"+QUOTATION+")" + NEW_LINE  + 
"              enter_critical" + NEW_LINE  + 
"              if jog_thrd != 0:" + NEW_LINE  + 
"                textmsg("+QUOTATION+"stopping jogging - killing jog_thrd"+QUOTATION+")" + NEW_LINE  + 
"                kill jog_thrd" + NEW_LINE  + 
"                jog_thrd = 0" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              exit_critical" + NEW_LINE  + 
"              stopl(1.2)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"stop_jog done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 43:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_forward_kinematics_default"+QUOTATION+")" + NEW_LINE  + 
"              forward_kin = get_forward_kin()" + NEW_LINE  + 
"              textmsg(forward_kin)" + NEW_LINE  + 
"              pose_to_output_float_registers(0, forward_kin)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_forward_kinematics_default done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 44:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_forward_kinematics_args"+QUOTATION+")" + NEW_LINE  + 
"              q = q_from_input_float_registers(0)" + NEW_LINE  + 
"              tcp_offset = pose_from_input_float_registers(6)" + NEW_LINE  + 
"              forward_kin = get_forward_kin(q, tcp_offset)" + NEW_LINE  + 
"              textmsg(forward_kin)" + NEW_LINE  + 
"              pose_to_output_float_registers(0, forward_kin)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_forward_kinematics_args done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 45:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"move_path"+QUOTATION+")" + NEW_LINE  + 
"              async = read_input_integer_reg(1)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"async: "+QUOTATION+", async)" + NEW_LINE  + 
"              stop_async_move()" + NEW_LINE  + 
"              if async == 1:" + NEW_LINE  + 
"                  enter_critical" + NEW_LINE  + 
"                  move_type = 4 # move_path" + NEW_LINE  + 
"                  exit_critical" + NEW_LINE  + 
"                  move_thrd = run move_thread()" + NEW_LINE  + 
"              else:" + NEW_LINE  + 
"                  exec_move_path()" + NEW_LINE  + 
"                  textmsg("+QUOTATION+"move_path done"+QUOTATION+")" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"        elif cmd == 46:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_inverse_kin_default"+QUOTATION+")" + NEW_LINE  + 
"              x = pose_from_input_float_registers(0)" + NEW_LINE  + 
"              q = get_inverse_kin(x)" + NEW_LINE  + 
"              q_to_output_float_registers(0, q)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_inverse_kin_default done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 47:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"is_steady"+QUOTATION+")" + NEW_LINE  + 
"              robot_is_steady = is_steady()" + NEW_LINE  + 
"              if robot_is_steady == True:" + NEW_LINE  + 
"                 write_output_integer_reg(1, 1)" + NEW_LINE  + 
"              else:" + NEW_LINE  + 
"                 write_output_integer_reg(1, 0)" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              textmsg("+QUOTATION+"is_steady done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 51:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"move_until_contact"+QUOTATION+")" + NEW_LINE  + 
"$5.4          xd = q_from_input_float_registers(0)" + NEW_LINE  + 
"$5.4          contact_dir = pose_from_input_float_registers(6)" + NEW_LINE  + 
"$5.4          acc = read_input_float_reg(12)" + NEW_LINE  + 
"$5.4          while True:" + NEW_LINE  + 
"$5.4             step_back = tool_contact(direction=contact_dir)" + NEW_LINE  + 
"$5.4             if step_back <= 0:" + NEW_LINE  + 
"$5.4                # Continue moving with specified speed vector" + NEW_LINE  + 
"$5.4                speedl(xd, acc, t=get_steptime())" + NEW_LINE  + 
"$5.4             else:" + NEW_LINE  + 
"$5.4                # Contact detected!" + NEW_LINE  + 
"$5.4                # Get q for when the contact was first seen" + NEW_LINE  + 
"$5.4                q = get_actual_joint_positions_history(step_back)" + NEW_LINE  + 
"$5.4                # Stop the movement" + NEW_LINE  + 
"$5.4                stopl(3)" + NEW_LINE  + 
"$5.4                # Move to the initial contact point" + NEW_LINE  + 
"$5.4                movel(q)" + NEW_LINE  + 
"$5.4                break" + NEW_LINE  + 
"$5.4             end" + NEW_LINE  + 
"$5.4          end" + NEW_LINE  + 
"              textmsg("+QUOTATION+"move_until_contact done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 52:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"freedrive_mode"+QUOTATION+")" + NEW_LINE  + 
"$5.10         free_axes = [1, 1, 1, 1, 1, 1]" + NEW_LINE  + 
"$5.10         free_axes[0] = read_input_integer_reg(1)" + NEW_LINE  + 
"$5.10         free_axes[1] = read_input_integer_reg(2)" + NEW_LINE  + 
"$5.10         free_axes[2] = read_input_integer_reg(3)" + NEW_LINE  + 
"$5.10         free_axes[3] = read_input_integer_reg(4)" + NEW_LINE  + 
"$5.10         free_axes[4] = read_input_integer_reg(5)" + NEW_LINE  + 
"$5.10         free_axes[5] = read_input_integer_reg(6)" + NEW_LINE  + 
"" + NEW_LINE  + 
"$5.10         freedrive_feature = pose_from_input_float_registers(0)" + NEW_LINE  + 
"$5.10         freedrive_mode(freeAxes=free_axes, feature=freedrive_feature)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"freedrive_mode done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 53:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"end_freedrive_mode"+QUOTATION+")" + NEW_LINE  + 
"$5.10         end_freedrive_mode()" + NEW_LINE  + 
"              textmsg("+QUOTATION+"end_freedrive_mode done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 54:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_freedrive_status"+QUOTATION+")" + NEW_LINE  + 
"$5.10         freedrive_status = get_freedrive_status()" + NEW_LINE  + 
"$5.10         write_output_integer_reg(1, freedrive_status)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_freedrive_status done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 56:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"ft_rtde_input_enable"+QUOTATION+")" + NEW_LINE  + 
"              enable = False" + NEW_LINE  + 
"              enable_val = read_input_integer_reg(1)" + NEW_LINE  + 
"              if enable_val == 1:" + NEW_LINE  + 
"                  enable = True" + NEW_LINE  + 
"              elif enable_val == 0:" + NEW_LINE  + 
"                  enable = False" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              sensor_mass = read_input_float_reg(0)" + NEW_LINE  + 
"              sensor_measuring_offset = [0, 0, 0]" + NEW_LINE  + 
"              sensor_measuring_offset[0] = read_input_float_reg(1)" + NEW_LINE  + 
"              sensor_measuring_offset[1] = read_input_float_reg(2)" + NEW_LINE  + 
"              sensor_measuring_offset[2] = read_input_float_reg(3)" + NEW_LINE  + 
"              sensor_cog = [0, 0, 0]" + NEW_LINE  + 
"              sensor_cog[0] = read_input_float_reg(4)" + NEW_LINE  + 
"              sensor_cog[1] = read_input_float_reg(5)" + NEW_LINE  + 
"              sensor_cog[2] = read_input_float_reg(6)" + NEW_LINE  + 
"$5.9          ft_rtde_input_enable(enable, sensor_mass, sensor_measuring_offset, sensor_cog)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"ft_rtde_input_enable done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 57:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"enable_external_ft_sensor"+QUOTATION+")" + NEW_LINE  + 
"              enable = False" + NEW_LINE  + 
"              enable_val = read_input_integer_reg(1)" + NEW_LINE  + 
"              if enable_val == 1:" + NEW_LINE  + 
"                  enable = True" + NEW_LINE  + 
"              elif enable_val == 0:" + NEW_LINE  + 
"                  enable = False" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              sensor_mass = read_input_float_reg(0)" + NEW_LINE  + 
"              sensor_measuring_offset = [0, 0, 0]" + NEW_LINE  + 
"              sensor_measuring_offset[0] = read_input_float_reg(1)" + NEW_LINE  + 
"              sensor_measuring_offset[1] = read_input_float_reg(2)" + NEW_LINE  + 
"              sensor_measuring_offset[2] = read_input_float_reg(3)" + NEW_LINE  + 
"              sensor_cog = [0, 0, 0]" + NEW_LINE  + 
"              sensor_cog[0] = read_input_float_reg(4)" + NEW_LINE  + 
"              sensor_cog[1] = read_input_float_reg(5)" + NEW_LINE  + 
"              sensor_cog[2] = read_input_float_reg(6)" + NEW_LINE  + 
"              enable_external_ft_sensor(enable, sensor_mass, sensor_measuring_offset, sensor_cog)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"enable_external_ft_sensor done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 58:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_actual_tool_flange_pose"+QUOTATION+")" + NEW_LINE  + 
"              flange_pose = get_actual_tool_flange_pose()" + NEW_LINE  + 
"              q_to_output_float_registers(0, flange_pose)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_actual_tool_flange_pose done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 59:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"set_gravity"+QUOTATION+")" + NEW_LINE  + 
"              gravity_direction = [0, 0, 0]" + NEW_LINE  + 
"              gravity_direction[0] = read_input_float_reg(0)" + NEW_LINE  + 
"              gravity_direction[1] = read_input_float_reg(1)" + NEW_LINE  + 
"              gravity_direction[2] = read_input_float_reg(2)" + NEW_LINE  + 
"              set_gravity(gravity_direction)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"set_gravity done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 60:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_inverse_kin_has_solution_default"+QUOTATION+")" + NEW_LINE  + 
"$5.10|3.15    x = pose_from_input_float_registers(0)" + NEW_LINE  + 
"$5.10|3.15    has_solution = get_inverse_kin_has_solution(x)" + NEW_LINE  + 
"$5.10|3.15    if has_solution == True:" + NEW_LINE  + 
"$5.10|3.15       write_output_integer_reg(1, 1)" + NEW_LINE  + 
"$5.10|3.15    else:" + NEW_LINE  + 
"$5.10|3.15       write_output_integer_reg(1, 0)" + NEW_LINE  + 
"$5.10|3.15    end" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_inverse_kin_has_solution_default done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 61:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_inverse_kin_has_solution_args"+QUOTATION+")" + NEW_LINE  + 
"$5.10|3.15    x = pose_from_input_float_registers(0)" + NEW_LINE  + 
"$5.10|3.15    qnear = q_from_input_float_registers(6)" + NEW_LINE  + 
"$5.10|3.15    maxPositionError = read_input_float_reg(12)" + NEW_LINE  + 
"$5.10|3.15    maxOrientationError = read_input_float_reg(13)" + NEW_LINE  + 
"" + NEW_LINE  + 
"$5.10|3.15    has_solution = get_inverse_kin_has_solution(x, qnear, maxPositionError, maxOrientationError)" + NEW_LINE  + 
"$5.10|3.15    if has_solution == True:" + NEW_LINE  + 
"$5.10|3.15        write_output_integer_reg(1, 1)" + NEW_LINE  + 
"$5.10|3.15    else:" + NEW_LINE  + 
"$5.10|3.15        write_output_integer_reg(1, 0)" + NEW_LINE  + 
"$5.10|3.15    end" + NEW_LINE  + 
"              textmsg("+QUOTATION+"get_inverse_kin_has_solution_args done"+QUOTATION+")" + NEW_LINE  + 
"            elif cmd == 62:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"start_contact_detection"+QUOTATION+")" + NEW_LINE  + 
"              enter_critical" + NEW_LINE  + 
"              if contact_thrd != 0:" + NEW_LINE  + 
"                  textmsg("+QUOTATION+"killing contact_thrd"+QUOTATION+")" + NEW_LINE  + 
"                  kill contact_thrd" + NEW_LINE  + 
"                  contact_thrd = 0" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              exit_critical" + NEW_LINE  + 
"              contact_direction = pose_from_input_float_registers(0)" + NEW_LINE  + 
"              contact_thrd = run contact_thread()" + NEW_LINE  + 
"              textmsg("+QUOTATION+"start_contact_detection done"+QUOTATION+")" + NEW_LINE  + 
"          elif cmd == 63:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"stop_contact_detection"+QUOTATION+")" + NEW_LINE  + 
"              enter_critical" + NEW_LINE  + 
"              if contact_thrd != 0:" + NEW_LINE  + 
"                  textmsg("+QUOTATION+"killing contact_thrd"+QUOTATION+")" + NEW_LINE  + 
"                  kill contact_thrd" + NEW_LINE  + 
"                  contact_thrd = 0" + NEW_LINE  + 
"              end" + NEW_LINE  + 
"              exit_critical" + NEW_LINE  + 
"              write_output_integer_reg(1, contact_step_back)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"stop_contact_detection done"+QUOTATION+")" + NEW_LINE  + 
"          elif cmd == 64:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"read_contact_detection"+QUOTATION+")" + NEW_LINE  + 
"              enter_critical" + NEW_LINE  + 
"              step_back = contact_step_back" + NEW_LINE  + 
"              exit_critical" + NEW_LINE  + 
"              write_output_integer_reg(1, step_back)" + NEW_LINE  + 
"              textmsg("+QUOTATION+"read_contact_detection done"+QUOTATION+")" + NEW_LINE  + 
"          elif cmd == 254: # internal command" + NEW_LINE  + 
"              textmsg("+QUOTATION+"cmd == 254 - contact detected"+QUOTATION+") " + NEW_LINE  + 
"$5.4          stop_async_move()" + NEW_LINE  + 
"$5.4          # Get q for when the contact was first seen" + NEW_LINE  + 
"$5.4          q = get_actual_joint_positions_history(contact_step_back)" + NEW_LINE  + 
"$5.4          # Stop the movement" + NEW_LINE  + 
"$5.4          stopl(3)" + NEW_LINE  + 
"$5.4          # Move to the initial contact point" + NEW_LINE  + 
"$5.4          movel(q)" + NEW_LINE  + 
"$5.4          textmsg("+QUOTATION+"cmd == 254 - contact detected done"+QUOTATION+")" + NEW_LINE  + 
"        elif cmd == 255:" + NEW_LINE  + 
"              textmsg("+QUOTATION+"Received stop script!"+QUOTATION+")" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        if cmd != 255:" + NEW_LINE  + 
"              signal_done_with_cmd()" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"" + NEW_LINE  + 
"        return cmd != 255" + NEW_LINE  + 
"    end" + NEW_LINE  + 
"" + NEW_LINE  + 
"# HEADER_END" + NEW_LINE  + 
"" + NEW_LINE  + 
"# NODE_CONTROL_LOOP_BEGINS" + NEW_LINE  + 
"" + NEW_LINE  + 
"        ###################################" + NEW_LINE  + 
"        # RTDE Control script - Main loop #" + NEW_LINE  + 
"        ###################################" + NEW_LINE  + 
"        textmsg("+QUOTATION+"RTDE Control Script Loaded (06.06.2023-13:52)"+QUOTATION+")" + NEW_LINE  + 
"" + NEW_LINE  + 
"        # Initialize gain and damping for force mode to a more stable default" + NEW_LINE  + 
"$5.0    force_mode_set_gain_scaling(0.5)" + NEW_LINE  + 
"$3.5    force_mode_set_damping(0.025)" + NEW_LINE  + 
"" + NEW_LINE  + 
"        keep_running = True" + NEW_LINE  + 
"        executing_cmd = False" + NEW_LINE  + 
"        reset_async_progress()" + NEW_LINE  + 
"        signal_ready()" + NEW_LINE  + 
"" + NEW_LINE  + 
"        while keep_running:" + NEW_LINE  + 
"            cmd = rtde_cmd()" + NEW_LINE  + 
"            if cmd == 24 or cmd == 11 or cmd == 9 or cmd == 10 or cmd == 6 or cmd == 25 or cmd == 26 or cmd == 27 or cmd == 38 or cmd == 58:" + NEW_LINE  + 
"                # for realtime commands simply process and signal ready." + NEW_LINE  + 
"                keep_running = process_cmd(cmd)" + NEW_LINE  + 
"                signal_ready()" + NEW_LINE  + 
"            else:" + NEW_LINE  + 
"                # regular mode" + NEW_LINE  + 
"                if cmd == 0:" + NEW_LINE  + 
"                    executing_cmd = False" + NEW_LINE  + 
"                    signal_ready()" + NEW_LINE  + 
"                else:" + NEW_LINE  + 
"                    if not executing_cmd:" + NEW_LINE  + 
"                        keep_running = process_cmd(cmd)" + NEW_LINE  + 
"                    end" + NEW_LINE  + 
"                    executing_cmd = True" + NEW_LINE  + 
"                end" + NEW_LINE  + 
"            end" + NEW_LINE  + 
"" + NEW_LINE  + 
"            sync()" + NEW_LINE  + 
"        end" + NEW_LINE  + 
"        textmsg("+QUOTATION+"RTDE Control Script Terminated"+QUOTATION+")" + NEW_LINE  + 
"# NODE_CONTROL_LOOP_ENDS" + NEW_LINE  + 
"";