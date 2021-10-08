#!/usr/bin/perl
$first = 1;
$showRms = 0;

open(F1, "<$ARGV[0]") || die "Couldn't open $ARGV[0]";
open(F2, "<$ARGV[1]") || die "Couldn't open $ARGV[1]";


while(<F1>) {
	$l2 = <F2>;
	@tokens1 = split ' ';
	@tokens2 = split(' ', $l2);
	print $tokens1[0] . " ";
	for my $n (1 .. $#tokens2) {
		print $tokens1[$n] - $tokens2[$n] . " ";
	}
	print "\n";
}


