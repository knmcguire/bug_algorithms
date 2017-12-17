close all, clear all, clc


load /home/knmcguire/Documents/experiments/bug_algorithms/results/results_12-15-2017_14-14.mat

bug_names = {'wf', 'com_bug', 'bug_2','alg_1', 'alg_2', 'i_bug', 'blind_bug'};

fitness = zeros(length(results.environment),length(bug_names))
time = zeros(length(results.environment),length(bug_names))
mean_gradient = zeros(length(results.environment),length(bug_names))
for it = 1:length(results.environment)
    
    for itk = 1:length(results.environment(it).bug)
        
    bug_temp = results.environment(it).bug;
    index = find(strcmp({bug_temp.bug_name},bug_names{itk}));
        if(~isempty(index))
        fitness(it,itk) = results.environment(it).bug(index).fitness(1);
        indices_time = find(results.environment(it).bug(index).distances<1);
        if(isempty(indices_time))
           time(it,itk) = NaN;

        else
          time(it,itk) = indices_time(1)

        end
        
        mean_gradient(it,itk) =  mean(diff(results.environment(it).bug(index).distances));
        end

    end
    
end
   
figure,boxplot(fitness)
figure,boxplot(time)
figure,boxplot(mean_gradient)
