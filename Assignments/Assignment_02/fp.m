% Developer: Lucas Rath (https://github.com/lucasrm25)

classdef fp
    properties
    end
    
    methods(Static)
        
        % varargin{1}: figure handle to be saved. If not specified -> gcf
        function savefig(fname,varargin)
            figformat = 'epsc';
            folder = fullfile(pwd,'images');
            if nargin <=1
                fighandle = gcf;
            else
                fighandle = varargin{1};
            end
            if ~ exist(folder,'dir'), mkdir(folder); end
            set(gca,'LooseInset',get(gca,'TightInset'))
            saveas(fighandle, fullfile(folder,fname),figformat)
        end
        
        function fig = f()
            fig = figure('Color','white');
            hold on, grid on;
        end
        
        function latexcode = m2latex(matrix)
            if ~isa(matrix,'sym')
                matrix = sym(matrix);
            end
            latexcode = latex(matrix)
            clipboard('copy',latexcode);
        end
        
        % varargin{1}: line opacity
        function cl = getColor(n, varargin)
            colrs = lines(20);
            if nargin >= 2
                opacity = varargin{1};
            end
            cl = [colrs(n,:), opacity];
        end
    end
end

