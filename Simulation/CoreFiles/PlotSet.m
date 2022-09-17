function Out = PlotSet()

set(groot,'defaultLineLineWidth',1);                            %	Set Default Line Weight
set(groot,'defaultAxesFontName','Times New Roman');             %   Set Axis Font
set(groot,'defaultAxesFontSize',15);                            %   Set Axis Font Size
set(groot,'defaultAxesLabelFontSizeMultiplier',1);              %   Set Axis Label Font
% set(groot,'defaultFigurePosition',[200 100 800 500]);           %   Set Figure Default Position
set(gcf,'PaperUnits','inches','PaperSize',[700 400]./96);       %   Set Figure Size
% set(gca,'XTickLabel',{'0','10','20','30','40','50','60'});    %	Set X-Axis Coordinate interval
close all;

end