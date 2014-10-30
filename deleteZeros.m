function [W,Xi,landmarks] = deleteZeros(W,Xi,N,poses,landmarks)

%delete zero columns and rows which correspond to unseen landmarks

landnum = size(landmarks,2);
for i=landnum:-1:1
    b = 2*(poses+i);
    if (~any(W(b,:))&&(~any(W(:,b))))
          W = [W(1:b-2,:); W(b+1:size(W,1),:)];    
          W = [W(:,1:b-2) W(:,b+1:size(W,2))];
          Xi = [Xi(1:b-2) ; Xi(b+1:size(Xi,1))];   
          landmarks = [landmarks(:, 1:i-1) landmarks(:,i+1:size(landmarks,2))];
    end
end


end