clear all, clc

load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_01-15-2018_09-07.mat
results1 = results;
clear results
load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_01-15-2018_17-19.mat
results2 = results;
clear results
load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_01-16-2018_16-28.mat
results3 = results;
clear results
load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_01-17-2018_09-04.mat
results4 = results;
clear results
load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_01-22-2018_03-46.mat
results5 = results;
clear results

%results1.environment=rmfield(results1.environment,'init_position');
results.environment = [results1.environment, results2.environment, results3.environment, results4.environment, results5.environment];

clear results1 results2 results3 results4 results5


bug_names = {'wf', 'com_bug', 'bug_2','alg_1', 'alg_2', 'i_bug','blind_bug'};


reached_goal = zeros(length(results.environment)-1,length(bug_names),4);
lenght_trajectory = zeros(length(results.environment)-1,length(bug_names),4);
lenght_trajectory_percentage = zeros(4,length(results.environment)-1,length(bug_names));

optimal_path_length_per_environment=0;
se = offsetstrel('ball',3,3);
for it = 1:length(results.environment)
    
    
    it
    img_dilated = results.environment(it).img;%imdilate(results.environment(it).img,se);
    
  %  imwrite(img_dilated,['img_used_in_litsurvey/rand_env_lit_',num2str(it)],'PNG')
    
    did_all_bugs_make_it = 1;
    
    %optimal_path_per_environment = astar_on_environment(img_dilated,[1 1],[17 17]);
    %optimal_path_per_environment = astar_on_environment(img_dilated,[1 1],[12 12]);
    optimal_path_per_environment = astar_on_environment(img_dilated,[2 2],[11 11]);
    
    diff_trajectory = diff(optimal_path_per_environment);
    
    
    
    if ~isempty(diff_trajectory)
        optimal_path_length_per_environment = sum(sqrt(diff_trajectory(:,1).^2+diff_trajectory(:,2).^2))/10;
        
        
        for itn = 1:length(results.environment(it).noise)
            for itk = 1:length(results.environment(it).noise(itn).bug)
                
                bug_temp = results.environment(it).noise(itn).bug;
                index = find(strcmp({bug_temp.bug_name},bug_names{itk}));
                
                if(~isempty(index))
                    fitness(it,itk) = results.environment(it).noise(itn).bug(index).fitness(1);
                    indices_time = find(results.environment(it).noise(itn).bug(index).distances<1.0);
                    if indices_time > 3001
                        indices_time=[]
                    end
                    
                    %check of bugs toch bijna de goal hadden gehaald en
                    %corriceer
                    if(~isempty(indices_time))
                        results.environment(it).noise(itn).bug(index).distances(indices_time(1):end)=[];
                        results.environment(it).noise(itn).bug(index).trajectory(indices_time(1):end,:)=[];
                    end
                    [value_distance, min_index] = min(results.environment(it).noise(itn).bug(index).distances);
                    min_index = min(3000,length(results.environment(it).noise(itn).bug(index).distances));%length(results.environment(it).noise(itn).bug(index).distances);
                    
                    
                    diff_trajectory = diff(results.environment(it).noise(itn).bug(index).trajectory);
                    
                    
                    
                    temp_lenght_trajectory = sum(sqrt(diff_trajectory(1:min_index-1,1).^2+diff_trajectory(1:min_index-1,2).^2))+value_distance;
                    lenght_trajectory(it,itk,itn)= temp_lenght_trajectory;
                    lenght_trajectory_percentage(itn,it,itk)= temp_lenght_trajectory/optimal_path_length_per_environment;
                    
                    if(isempty(indices_time))
                        %                     lenght_trajectory(it,:) = NaN;
                        %lenght_trajectory_percentage(it,itk,itn) = NaN;
                        reached_goal(it,itk,itn) = 0;
                        did_all_bugs_make_it = 0;
                    else
                        reached_goal(it,itk,itn) = 1;
                    end
                    
                end
                
            end
            
            
        end
        
    else
        disp("A* produced an error")
        time(it,:) = NaN;
        lenght_trajectory(it,:) = NaN;
        lenght_trajectory_percentage(it,:) = NaN;
    end
    
    
end


%figure,boxplot(lenght_trajectory, 'Labels',bug_names)
% figure,boxplot(lenght_trajectory_percentage(:,1:end-1), 'Labels',bug_names(1:end-1))

bug_names = {'WF', 'Combug', 'Bug2','Alg1', 'Alg2', 'Ibug','blind_bug'};

tempmat = matfile('temp.mat')
tempresult = tempmat.savetemp;
figure,
%aboxplot(tempresult, 'colorgrad','empty', 'WidthL',0.2)
hold on
temp_lenghth_trajectory = zeros(5,200,5)
temp_lenghth_trajectory(2:5,:,:)= lenght_trajectory_percentage(:,1:200,2:end-1);
temp_lenghth_trajectory(1,:,:) = tempresult;
aboxplot(temp_lenghth_trajectory)
set(gca,'xticklabel',bug_names(2:end-1))
ylabel('Trajectory bug / Trajectory A*')
ylim([0 10])
legend('\sigma = 0.00','\sigma = 0.05','\sigma =0.1','\sigma =0.15','\sigma =0.25')
hold on,



figure,

tempmat = matfile('temp2.mat')
tempresult = tempmat.savetemp;
for itn = 1:4
    for itk = 1:6
        reached_goal_bar(itk,itn) = 100*sum(reached_goal(:,itk,itn))/length(results.environment);
    end
end

reached_goal_bar_temp = zeros(5,5)
reached_goal_bar_temp(:,1) = tempresult
reached_goal_bar_temp(:,2:end) = reached_goal_bar(2:6,:)

bar_handle = bar(reached_goal_bar_temp,'grouped');
cmap = colorgrad(5,'blue_down');

for i = 1:5
    set(bar_handle(i),'FaceColor',cmap(i,:))
end

hold on
set(gca, 'XTickLabel',bug_names(2:end-1),'DefaultTextInterpreter', 'none')
ylabel('bugs made to goal [%]')



% figure,
% aboxplot(lenght_trajectory_percentage(1,:,1:end-1))
% set(gca,'xticklabel',bug_names(1:end-1))
% ylabel('Trajectory bug / Trajectory A*')
%
% %legend('\sigma = 0.0','\sigma =0.01','\sigma =0.25','\sigma =0.5')

% figure,

% for itn = 1:1
%     for itk = 1:6
%         reached_goal_bar(itk,itn) = sum(reached_goal(:,itk,itn))/length(results.environment);
%
%     end
% end
% cmap = colorgrad(1,'blue_down');
%
% bar_handle = bar(reached_goal_bar(:,1),'FaceColor',cmap(1,:));

% for i = 1:1
% set(bar_handle(i),'FaceColor',cmap(i,:))
% end

% hold on
% set(gca, 'XTickLabel',bug_names(1:end-1),'DefaultTextInterpreter', 'none')
% ylabel('bugs made to goal [%]')
%


%%
env_number = 1;
figure,
bug_number = 5
voorbeelden = 123%[32 56 105 122 123 140 188] 
noise = [0.05,0.1,0.15,0.2];

bug_names = {'WF', 'Combug', 'Bug2','Alg1', 'Alg2', 'Ibug','blind_bug'};

for env_number = 123
    env_number
    for it = 1:length(results.environment(env_number).noise)
        subplot(2,2,it),imshow(imresize(results.environment(env_number).img',2))
        hold on
        if(isfield(results.environment(env_number),'init_position'))
            plot(20*(results.environment(env_number).init_position(:,2)+7),20*(results.environment(env_number).init_position(:,1)-1+7), 'o')
        else
            plot(20*[1.5 11],20*[1.5 11], 'o')
        end
        
        hold on, plot(20*(results.environment(env_number).noise(it).bug(bug_number).trajectory(2:end,1)+7),20*(results.environment(env_number).noise(it).bug(bug_number).trajectory(2:end,2)+7),'g','LineWidth',1)
        
        title(['Alg2 ', num2str(noise(it)), ' (',...
            num2str(size(results.environment(env_number).noise(it).bug(bug_number).trajectory,1)/10), ' sec)'],'Interpreter', 'none')

        if reached_goal(env_number,bug_number,it) && it == 4

            
        end
    end
    
end