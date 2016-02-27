#!/usr/bin/env python
"""
Scraper for Doug McKenzie Jazz Piano MIDI recordings
"""
import urllib2
import urllib
from BeautifulSoup import BeautifulSoup


for i in xrange(1,14):
	url = "http://midkar.com/jazz/jazz_%02d" % i
	req = urllib2.Request(url)
	response = urllib2.urlopen(req)
	page_contents = response.read()

	base_curl_url = "http://midkar.com/jazz/"

	soup = BeautifulSoup(page_contents)
	for link in soup.findAll('a', href=True):
		if link['href'].endswith('.mid'):
			print "Scraping " + link['href']
			full_url = base_curl_url + urllib.quote(link['href'])
			read_file = urllib.URLopener()
			try:
				read_file.retrieve(full_url, "midkar/" + link['href'])
			except IOError, e:
				print e





