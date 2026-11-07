fmsPages["ATCREJECTDUETO"]=createPage("ATCREJECTDUETO")
fmsPages["ATCREJECTDUETO"].getPage=function(self,pgNo,fmsID)--dynamic pages need to be this way
    return{

"      REJECT DUE TO     ",
"                        ",		               
"<                       ",
"                        ",
"<                       ",
"                        ",
"<                       ",
"                        ",
"                        ",
"<                       ",
"                        ",
"------------------------",
"<UPLINK          VERIFY>" 
    }
end

fmsPages["ATCREJECTDUETO"].getSmallPage=function(self,pgNo,fmsID)--dynamic pages need to be this way
    return{
"                    1/1 ",
" DUE TO           DUE TO",		               
"<PERFORMANCE    WEATHER>",
" FREE TEXT              ",
"                        ",
"                        ",
"                        ",
"                        ",
"                        ",
"                RESPONSE",
"                        ",
"                        ",
"                        "
}
end

  
fmsFunctionsDefs["ATCREJECTDUETO"]={}

