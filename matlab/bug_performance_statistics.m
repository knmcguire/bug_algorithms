close all, clear all, clc
%  folderpath = '/home/knmcguire/Documents/Software/catkin_ws/src'
%  rosgenmsg(folderpath);

rosinit

bug_names = {'wf', 'com_bug', 'bug_2','alg_1', 'alg_2', 'i_bug', 'blind_bug'};
switch_bug_pub = rospublisher('/switch_bug','std_msgs/String');
environment_random_pub = rospublisher('/random_environment','std_msgs/Bool');

msg = rosmessage(switch_bug_pub);
msg_env = rosmessage(environment_random_pub);

amount_of_bugs = 3;
amount_of_environments = 9;
for itk = 1:amount_of_environments
    msg_env.Data = 1;
    send(environment_random_pub,msg_env);
    for it = 1:amount_of_bugs
        
        msg.Data = bug_names{it};
        send(switch_bug_pub,msg) ;
        disp("Send out switch bug message")
        
        pause(5)
        sub = rossubscriber('/finished_sim_matlab');
        receive(sub);
        
        disp("Simulation is Finished")
        pause(3)

        results.environment(itk).bug(it).bug_name = bug_names{it};
        results.environment(itk).bug(it).trajectory = csvread("/home/knmcguire/.ros/trajectory.txt");
        results.environment(itk).bug(it).fitness = csvread("/home/knmcguire/.ros/fitness.txt");
        results.environment(itk).bug(it).distances = csvread("/home/knmcguire/.ros/distances.txt");

    end
    figure(1)
    results.environment(itk).img= imread('/home/knmcguire/.ros/environment.png');
    if itk < 9
        subplot(3,3,itk),imshow(imresize(results.environment(itk).img',2))
    end
    for it = 1:amount_of_bugs
        hold on, plot(20*(results.environment(itk).bug(it).trajectory(:,2)+5),20*(results.environment(itk).bug(it).trajectory(:,1)+5))
    end
    
    
end
Filename = sprintf('trajectories_%s.png', datestr(now,'mm-dd-yyyy_HH-MM'));
saveas(gcf,Filename)
Filename = sprintf('results_%s.mat', datestr(now,'mm-dd-yyyy_HH-MM'));
save(Filename, 'results')

rosshutdown