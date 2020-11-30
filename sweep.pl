#!/usr/bin/perl

$num = shift(@ARGV);
while(<STDIN>) {
	($param, $lo, $hi, $inc) = split(' ', $_);
	$lo{$param} = $lo;
	$hi{$param} = $hi;
	$inc{$param} = $inc;
	push(@params, $param);
}

foreach $it ( 1..10 ) { 
	foreach $t ( @params ) {
		print "$t\n";
		for ($v = $lo{$t}; $v <= $hi{$t}; $v = $v + $inc{$t}) {  
			$args = ",";
			foreach $p ( @params ) {
				if (exists $bestParam{$p} && ($t ne $p)) { 
					$args = $args . "$p=$bestParam{$p},";
				} 
				if ($t eq $p) {
					$args = $args . "$p=$v,";
				} 
			}
			$cmd = "./winglevlr_ubuntu --replay ./logs/AHRSD$num.DAT --debug \"$args\"";
			print $cmd . "...";
			($x, $result) = (`$cmd` =~ /#\s([0-9.+-]+)\s([0-9.+-]+)/ );
			print $result. "  ";
			if (!exists $best{$t} || $result <= $best{$t}) {
				print "\t<<=== BEST"; 
				$best{$t} = $result;
				$bestParam{$t} = $v;
				$bestCmd = $cmd;
			}
			print "\n";
		}
	}
}
print $bestCmd . "\n";
