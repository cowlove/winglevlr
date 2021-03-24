#!/usr/bin/perl
$first = 1;
$showRms = 0;

if ($ARGV[0] =~ /--rms/) {
	$showRms = 1;
	shift;
}

while(<>) {
	push @hist, $_;
	@tokens = split ' ';
	for my $n (0 .. $#tokens) {
		print $tokens[$n] - $last[$n] . " ";
	}
	@last = @tokens;
	print "\n";
}


