%% example
% s=DM_serial_CAN('COM5')
% s.f_moter_en_all()
% s.f_moter_go_zero_all()
% s.f_moter_disen_all()

% % f_read_can_id(obj)
% f_moter_en_all(obj)
% % f_pos_vel(obj, can_id, pos, vel)
% % f_moter_en(obj, '141')
% % f_moter_en(obj, '122')
% % f_pos_vel(obj, '140', 0, 0.5)
% f_moter_go_zero_all(obj)
% pause(5)
% f_moter_disen_all(obj)

classdef DM_serial_CAN< handle
    properties %成员属性块--开始
        dev
        can_id
        can_id_s
    end %成员属性块--结束

    methods %成员函数块--开始
        % 构造函数（初始化CAN接口）
        function obj = DM_serial_CAN(COM)  %%Point2D类的构造函数
            clc
            can_id = 199;
            can_id_s=0;
            obj.dev =serialport(COM,115200,"Timeout",0.01);
        end


        %% function below
        %% 读取can id
        function [can_id_s] = f_read_can_id (obj)
            udata = [0xAA, 0, 0, 0x08, 0, 0, 0x07, 0xff, 0x55, 0x00, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00];
            write(obj.dev,udata,"uint8")
        end

        %% 使能电机
        function [can_id_s] = f_moter_en(obj, can_id)
            % can_id_h = fix(can_id/100);
            % can_id_l = mod(can_id, 100);
            can_id_h = hex2dec(can_id(1));
            can_id_l = hex2dec(can_id(2:3));


            udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC];
            write(obj.dev,udata,"uint8")
        end

        %% 使能全部电机
        function [can_id_s] = f_moter_en_all(obj)
            f_moter_en(obj, '110'); pause(0.5)
            f_moter_en(obj, '111'); pause(0.5)
            f_moter_en(obj, '112'); pause(0.5)
            f_moter_en(obj, '120'); pause(0.5)
            f_moter_en(obj, '121'); pause(0.5)
            f_moter_en(obj, '122'); pause(0.5)
            f_moter_en(obj, '130'); pause(0.5)
            f_moter_en(obj, '131'); pause(0.5)
            f_moter_en(obj, '132'); pause(0.5)
            f_moter_en(obj, '140'); pause(0.5)
            f_moter_en(obj, '141'); pause(0.5)
            f_moter_en(obj, '142'); pause(0.5)
        end

        %% 失能电机
        function [can_id_s] = f_moter_disen(obj, can_id)
            % can_id_h = hex2dec(fix(can_id/100));
            % can_id_l = hex2dec(mod(can_id,100));
            can_id_h = hex2dec(can_id(1));
            can_id_l = hex2dec(can_id(2:3));

            udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD];
            write(obj.dev,udata,"uint8")
        end

        %% 失能全部电机
        function [can_id_s] = f_moter_disen_all(obj)
            f_moter_disen(obj, '110'); pause(0.5)
            f_moter_disen(obj, '111'); pause(0.5)
            f_moter_disen(obj, '112'); pause(0.5)
            f_moter_disen(obj, '120'); pause(0.5)
            f_moter_disen(obj, '121'); pause(0.5)
            f_moter_disen(obj, '122'); pause(0.5)
            f_moter_disen(obj, '130'); pause(0.5)
            f_moter_disen(obj, '131'); pause(0.5)
            f_moter_disen(obj, '132'); pause(0.5)
            f_moter_disen(obj, '140'); pause(0.5)
            f_moter_disen(obj, '141'); pause(0.5)
            f_moter_disen(obj, '142'); pause(0.5)
        end

        %% 保存零点
        function [can_id_s] = f_set_zero_point(obj, can_id)
            % can_id_h = hex2dec(fix(can_id/100));
            % can_id_l = hex2dec(mod(can_id,100));
            can_id_h = hex2dec(can_id(1));
            can_id_l = hex2dec(can_id(2:3));

            udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE];
            write(obj.dev,udata,"uint8")
        end

        %% 清除电机状态
        function [can_id_s] = f_clear_moter(obj, can_id)
            % can_id_h = hex2dec(fix(can_id/100));
            % can_id_l = hex2dec(mod(can_id,100));
            can_id_h = hex2dec(can_id(1));
            can_id_l = hex2dec(can_id(2:3));

            udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB];
            write(obj.dev,udata,"uint8")
        end
        %% 位置速度
        function [can_id_s] = f_pos_vel(obj, can_id, pos, vel)
            % can_id_h = fix(can_id/100);
            % can_id_l = mod(can_id, 100);
            can_id_h = hex2dec(can_id(1));
            can_id_l = hex2dec(can_id(2:3));

            q = quantizer('single');
            pos_hex = num2hex(q,pos);
            vel_hex = num2hex(q,vel);

            udata = [0xAA, 0, 0, 0x08, 0, 0, can_id_h, can_id_l, hex2dec(pos_hex(7:8)), hex2dec(pos_hex(5:6)), hex2dec(pos_hex(3:4)), hex2dec(pos_hex(1:2)), hex2dec(vel_hex(7:8)), hex2dec(vel_hex(5:6)), hex2dec(vel_hex(3:4)), hex2dec(vel_hex(1:2))]
            write(obj.dev,udata,"uint8")
        end

        %% 全部电机到零点
        function [can_id_s] = f_moter_go_zero_all(obj)
            f_pos_vel(obj, '110', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '120', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '130', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '140', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '111', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '121', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '131', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '141', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '112', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '122', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '132', 0, 0.5); pause(0.5)
            f_pos_vel(obj, '142', 0, 0.5); pause(0.5)

        end
    end
end
