function error=match_error(object1,object2,transform12)
object1_homo = [object1;ones(1,size(object1,2))];
object1 = transform12*object1_homo;
object1 = object1(1:3,:);
if size(object2,2)>size(object1,2)
    Md = createns(object1');
    [ RefIdx, Residuals ] = knnsearch( Md, object2' );
else
    Md = createns(object2');
    [ RefIdx, Residuals ] = knnsearch( Md, object1' );
end
error = sum(Residuals,'all')/size(Residuals,1);
end