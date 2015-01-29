#!/bin/perl

open(FD,"<data.hh");

open(OUT,">data.h");

print OUT "const unsigned short DATA[] = {\n";

while (<FD>) {                                 
  $line = $_;                                
  $line =~ s/ //g;                           
  @array = ( $line =~ m/..../g );            
  foreach (@array) {                         
    print OUT "0x".$_.",";                       
  }                                          
  print OUT "\n\t";                                
}

print OUT "0x0000};\n";

close(FD);
close(OUT);                                 